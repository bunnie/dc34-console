// bio.rs — REPL command that receives a BIO program over serial and runs it.

use core::fmt::Write;
use std::io::Read;
use std::io::Write as FsWrite;

use bao1x_api::bio::*;
use bao1x_api::bio_resources::*;
use bao1x_hal::bio::{Bio, CoreCsr};
use base64::{Engine as _, engine::general_purpose::STANDARD as B64};
use bytemuck::cast;
use dc34_api::{DC34_BIO, DC34_BIO_CLK, DC34_BIO_PINS, DC34_DICT};
use pddb::Pddb;

use crate::{CommonEnv, ShellCmdApi};

// -- Constants ----------------------------------------------------------------

const CHUNK_DATA_SIZE: usize = 64;
const CHUNK_INDEX_BYTES: usize = 2;
const CHUNK_CRC_BYTES: usize = 4;
/// Total decoded wire size per chunk.
const CHUNK_WIRE_SIZE: usize = CHUNK_INDEX_BYTES + CHUNK_DATA_SIZE + CHUNK_CRC_BYTES; // 70
/// 4096 bytes / 64 bytes per chunk = 64 chunks.
const NUM_CHUNKS: usize = 64;
/// Size of BIO memory in 32-bit words
const BIO_MEM_WORDS: usize = 1024;
const ALLOWED_PINS: [u8; 4] = [21, 22, 30, 31];

// -- Command state -------------------------------------------------------------
fn check_pins(pins: &mut Vec<u8>) {
    pins.retain(|p| ALLOWED_PINS.contains(p));
    pins.sort();
    pins.dedup();
}

pub struct BioLoader {
    /// Raw 64-byte payloads indexed by chunk number.
    /// `None` means that chunk has not yet been received.
    chunks: Vec<Option<[u8; CHUNK_DATA_SIZE]>>,

    /// How many distinct chunk slots are currently filled.
    received_count: usize,

    /// BIO clock config
    target_freq: u32,

    /// Pins used by this BIO
    pin_spec: Vec<u8>,

    /// Actual code
    code: Option<[u32; BIO_MEM_WORDS]>,

    pddb: Pddb,

    bio_ss: Bio,
    _fifo_handle: CoreHandle,
    fifo: CoreCsr,
    resource_grant: ResourceGrant,
}

impl BioLoader {
    pub fn new() -> Self {
        let bio_ss = Bio::new();
        let pddb = Pddb::new();
        // claim core resource and initialize it
        let resource_grant =
            bio_ss.claim_resources(&Self::resource_spec()).expect("Couldn't claim BIO resources");

        log::info!("using core: {:?}", resource_grant.cores[0]);

        // safety: fifo is stored in this object so they aren't Drop'd before the object is
        // destroyed
        let fifo_handle = unsafe { bio_ss.get_core_handle(Fifo::Fifo3) }
            .expect("Didn't get FIFO3 handle")
            .expect("Didn't get FIFO3 handle");
        let fifo = CoreCsr::from_handle(&fifo_handle);

        let mut loader = BioLoader {
            chunks: vec![None; NUM_CHUNKS],
            received_count: 0,
            target_freq: 350_000_000,
            pin_spec: vec![],
            code: None,
            pddb,
            bio_ss,
            _fifo_handle: fifo_handle,
            fifo,
            resource_grant,
        };
        match loader.reload() {
            Err(s) => {
                log::error!("Couldn't setup BIO: {:?}", s)
            }
            _ => log::info!("BIO setup from stored config!"),
        }

        loader
    }

    pub fn reload(&mut self) -> Result<(), String> {
        let config = {
            let mut clk_buf = [0u8; 4];
            let mut key = self
                .pddb
                .get(DC34_DICT, DC34_BIO_CLK, None, true, true, Some(4), None::<fn()>)
                .map_err(|_| "couldn't get PDDB key".to_string())?;
            let clk_len = key.read(&mut clk_buf).map_err(|_| "couldn't read key".to_string())?;
            self.target_freq = 350_000_000;
            if clk_len == 4 {
                let target_freq = u32::from_le_bytes(clk_buf);
                if target_freq > 0 && target_freq <= 350_000_000 {
                    self.target_freq = target_freq;
                    CoreConfig { clock_mode: bao1x_api::bio::ClockMode::TargetFreqInt(target_freq) }
                } else {
                    CoreConfig { clock_mode: bao1x_api::bio::ClockMode::TargetFreqInt(350_000_000) }
                }
            } else {
                CoreConfig { clock_mode: bao1x_api::bio::ClockMode::TargetFreqInt(350_000_000) }
            }
        };
        let mut pin_spec = Vec::<u8>::new();
        let mut key = self
            .pddb
            .get(DC34_DICT, DC34_BIO_PINS, None, true, true, Some(4), None::<fn()>)
            .map_err(|_| "couldn't get PDDB key".to_string())?;
        key.read_to_end(&mut pin_spec).map_err(|_| "couldn't read key".to_string())?;

        let mut code_buf = [0u8; 4096];
        let mut key = self
            .pddb
            .get(DC34_DICT, DC34_BIO, None, true, true, Some(4096), None::<fn()>)
            .map_err(|_| "couldn't get PDDB key".to_string())?;
        let code_len = key.read(&mut code_buf).map_err(|_| "couldn't read key".to_string())?;
        if code_len == 4096 {
            self.bio_ss
                .init_core(self.resource_grant.cores[0], (&code_buf, None), config)
                .map_err(|_| "Couldn't init core".to_string())?;
        }
        self.code = Some(cast(code_buf));

        check_pins(&mut pin_spec);
        for pin in &pin_spec {
            // claim pin resource - this only claims the resource, it does not configure it
            self.bio_ss
                .claim_dynamic_pin(*pin, &BioLoader::resource_spec().claimer)
                .map_err(|_| "can't claim pin".to_string())?;
            // now configure the claimed resource
            let mut io_config = IoConfig::default();
            io_config.mapped = 1 << (*pin as u32);

            io_config.mode = IoConfigMode::SetOnly;
            self.bio_ss.setup_io_config(io_config).unwrap();
        }
        self.pin_spec = pin_spec;

        self.bio_ss.set_core_run_state(&self.resource_grant, true);
        Ok(())
    }

    /// Return true when every chunk slot is filled.
    pub fn is_complete(&self) -> bool { self.received_count == NUM_CHUNKS }

    /// Assemble all received chunks into a `[u32; 1024]` code chunk.
    /// Panics if `is_complete()` is false — caller should check first.
    pub fn to_code(&self) -> [u32; BIO_MEM_WORDS] {
        assert!(self.is_complete(), "code not yet complete");
        let mut code = [0u32; BIO_MEM_WORDS];
        for (chunk_idx, slot) in self.chunks.iter().enumerate() {
            let data = slot.as_ref().unwrap();
            let word_base = chunk_idx * (CHUNK_DATA_SIZE / 4); // 16 words per chunk
            for w in 0..(CHUNK_DATA_SIZE / 4) {
                let o = w * 4;
                code[word_base + w] = u32::from_be_bytes([data[o], data[o + 1], data[o + 2], data[o + 3]]);
            }
        }
        code
    }

    /// Reset all state
    pub fn clear(&mut self) {
        for slot in self.chunks.iter_mut() {
            *slot = None;
        }
        self.received_count = 0;
    }
}

impl Resources for BioLoader {
    fn resource_spec() -> ResourceSpec {
        ResourceSpec {
            claimer: "BIO loader".to_string(),
            cores: vec![CoreRequirement::Any],
            fifos: vec![Fifo::Fifo3],
            static_pins: vec![],
            dynamic_pin_count: 4,
        }
    }
}

impl Drop for BioLoader {
    fn drop(&mut self) {
        for &core in self.resource_grant.cores.iter() {
            self.bio_ss.de_init_core(core).unwrap();
        }
        for pin in &self.pin_spec {
            self.bio_ss.release_dynamic_pin(*pin, &BioLoader::resource_spec().claimer).unwrap();
        }
        self.bio_ss.release_resources(self.resource_grant.grant_id).unwrap();
    }
}
// -- ShellCmdApi implementation ------------------------------------------------

impl<'a> ShellCmdApi<'a> for BioLoader {
    cmd_api!(image);

    fn process(&mut self, args: String, _env: &mut CommonEnv) -> Result<Option<String>, xous::Error> {
        let mut ret = String::new();
        if args == "clear" {
            self.clear();
            self.pddb.delete_key(DC34_DICT, DC34_BIO, None).ok();

            for &core in self.resource_grant.cores.iter() {
                self.bio_ss.de_init_core(core).unwrap();
            }
            for pin in &self.pin_spec {
                self.bio_ss.release_dynamic_pin(*pin, &BioLoader::resource_spec().claimer).unwrap();
            }
            self.bio_ss.release_resources(self.resource_grant.grant_id).unwrap();

            write!(ret, "CLEAR").unwrap();
            return Ok(Some(ret));
        }

        if args.starts_with("pin") {
            let arg_list: Vec<&str> = args.split_whitespace().collect();
            let mut pins = Vec::<u8>::new();
            if arg_list.len() >= 2 {
                for pin_str in &arg_list[1..] {
                    if let Ok(pin) = u8::from_str_radix(pin_str, 10) {
                        pins.push(pin);
                    }
                }
                let mut pin_key = self
                    .pddb
                    .get(DC34_DICT, DC34_BIO_PINS, None, true, true, Some(4), None::<fn()>)
                    .expect("couldn't get PDDB key");
                pin_key.write_all(&pins).ok();
            }
        }

        if args.starts_with("clk") {
            let arg_list: Vec<&str> = args.split_whitespace().collect();
            if arg_list.len() >= 2 {
                if let Ok(clk) = u32::from_str_radix(arg_list[1], 10) {
                    let mut clk_key = self
                        .pddb
                        .get(DC34_DICT, DC34_BIO_CLK, None, true, true, Some(4), None::<fn()>)
                        .expect("couldn't get PDDB key");
                    clk_key.write_all(&clk.to_le_bytes()).ok();
                }
            }
        }

        if args.starts_with("reload") {
            match self.reload() {
                Ok(_) => return Ok(Some("BIO load successful".to_string())),
                Err(e) => return Ok(Some(format!("BIO load error: {:?}", e))),
            }
        }

        // -- Decode base64 argument --------------------------------------------
        let b64 = args.trim();
        if b64.is_empty() {
            write!(ret, "ERR").unwrap();
            return Ok(Some(ret));
        }

        let decoded = match B64.decode(b64) {
            Ok(d) => d,
            Err(_) => {
                write!(ret, "ERR").unwrap();
                return Ok(Some(ret));
            }
        };

        if decoded.len() != CHUNK_WIRE_SIZE {
            write!(ret, "ERR").unwrap();
            return Ok(Some(ret));
        }

        // -- Parse fields ------------------------------------------------------
        let index = u16::from_be_bytes([decoded[0], decoded[1]]) as usize;
        // data lives at decoded[2..66]
        let received_crc = u32::from_be_bytes([decoded[66], decoded[67], decoded[68], decoded[69]]);

        // -- Verify CRC over (index bytes || data) -----------------------------
        let computed_crc = crc32fast::hash(&decoded[..CHUNK_INDEX_BYTES + CHUNK_DATA_SIZE]);
        if computed_crc != received_crc {
            write!(ret, "ERR").unwrap();
            return Ok(Some(ret));
        }

        // -- Bounds-check index ------------------------------------------------
        if index >= NUM_CHUNKS {
            write!(ret, "ERR").unwrap();
            return Ok(Some(ret));
        }

        // -- Store chunk (silent overwrite on duplicate) -----------------------
        let mut data_arr = [0u8; CHUNK_DATA_SIZE];
        data_arr.copy_from_slice(&decoded[CHUNK_INDEX_BYTES..CHUNK_INDEX_BYTES + CHUNK_DATA_SIZE]);

        let was_empty = self.chunks[index].is_none();
        self.chunks[index] = Some(data_arr);
        if was_empty {
            self.received_count += 1;
        }

        // -- Reply -------------------------------------------------------------
        if self.is_complete() {
            {
                let mut image_key = self
                    .pddb
                    .get(DC34_DICT, DC34_BIO, None, true, true, Some(4096), None::<fn()>)
                    .expect("couldn't get PDDB key");
                let words = self.to_code();
                let bytes: &[u8] = bytemuck::cast_slice(&words);
                image_key.write_all(bytes).ok();
            }
            self.clear();
            // trigger a reload using monkey-patched opcode
            let conn = _env.xns.request_connection_blocking("_Vault2_").unwrap();
            xous::send_message(conn, xous::Message::new_scalar(1024, 1, 0, 0, 0)).ok();
            write!(ret, "SUCCESS").unwrap();
        } else {
            write!(ret, "OK").unwrap();
        }

        Ok(Some(ret))
    }
}
