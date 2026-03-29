mod lightgenes;

use arbitrary_int::{Number, u5};
use bao1x_api::bio::*;
use bao1x_api::bio_resources::*;
use bao1x_hal::bio::{Bio, CoreCsr};
use bytemuck::{Pod, Zeroable};
use dc34_api::BadgeType;
use lightgenes::*;
use rand::Rng;

#[derive(Default, Pod, Zeroable, Copy, Clone, Debug)]
#[repr(C)]
pub struct Haploid {
    pub cd_period: u8,
    pub cd_rate: u8,
    pub cd_dir: u8,
    pub sat: u8,
    pub hue_ratedir: u8,
    pub hue_base: u8,
    pub hue_bound: u8,
    pub chaser: u8,
    pub nonlin: u8,
}
impl Haploid {
    pub fn from_rand() -> Self {
        let mut h = Haploid::default();
        h.cd_period = rand::thread_rng().gen_range(0..6);
        h.cd_rate = rand::thread_rng().gen();
        h.cd_dir = rand::thread_rng().gen();
        h.sat = rand::thread_rng().gen();
        h.hue_ratedir = rand::thread_rng().gen();
        h.hue_base = rand::thread_rng().gen();
        h.hue_bound = rand::thread_rng().gen();
        h.chaser = rand::thread_rng().gen();
        h.nonlin = rand::thread_rng().gen();
        h
    }

    pub fn from_type(badge_type: &BadgeType) -> Self {
        let mut h = Haploid::default();
        h.cd_period = rand::thread_rng().gen_range(0..badge_type.cd_period_max());
        h.cd_rate = rand::thread_rng().gen();
        h.cd_dir = rand::thread_rng().gen_range(badge_type.cd_dir_range());
        h.sat = rand::thread_rng().gen_range(badge_type.sat_range());
        h.hue_ratedir = rand::thread_rng().gen();
        h.hue_base = rand::thread_rng().gen_range(badge_type.hue_range());
        if *badge_type == BadgeType::Goon {
            // ensure that red is always part of the Goon pallette
            h.hue_base = 0;
        }
        h.hue_bound = rand::thread_rng().gen_range(h.hue_base..=badge_type.hue_range().end);
        if *badge_type == BadgeType::Uber {
            h.hue_bound = 255;
        }
        h.chaser = rand::thread_rng().gen_range(badge_type.chaser_range());
        h.nonlin = rand::thread_rng().gen_range(badge_type.nonlin_range());
        h
    }

    pub fn serialize(&self) -> Vec<u8> { bytemuck::bytes_of(self).to_vec() }

    pub fn deserialize(bytes: &[u8]) -> Option<Self> { bytemuck::try_from_bytes(bytes).ok().copied() }

    /// always returns a length-4 serialization, suitable for Xous args
    pub fn serialize_u32(&self) -> [u32; 4] {
        let bytes = bytemuck::bytes_of(self);
        let mut padded = [0u8; 16];
        padded[..bytes.len()].copy_from_slice(bytes);
        let mut out = [0u32; 4];
        for (i, chunk) in padded.chunks(4).enumerate() {
            out[i] = u32::from_le_bytes(chunk.try_into().unwrap());
        }
        out
    }

    /// gracefully handles 4 args by truncating the extra args that are just padding inserted by
    /// serialize_u32()
    pub fn deserialize_u32(words: &[u32]) -> Option<Self> {
        let bytes: Vec<u8> =
            words.iter().flat_map(|w| w.to_le_bytes()).take(std::mem::size_of::<Haploid>()).collect();
        bytemuck::try_from_bytes(&bytes).ok().copied()
    }
}

#[derive(Debug)]
pub struct Diploid(pub [Haploid; 2]);

impl Diploid {
    // computes the phenotypic expression of the diploid genome by blending the haploid pairs according
    // to rules that create dominant/recessive traits. In reality you don't get another strand of DNA, you get
    // proteins, but 'meh'. Close enough for computer science.
    pub fn phenotype(&self) -> Haploid {
        let mut e = Haploid {
            // add -> periodicity tends toward the mean, capped at 6 periods
            cd_period: ((self.0[0].cd_period + self.0[1].cd_period) / 2).min(6),
            // average -> rate tends toward the mean
            cd_rate: ((self.0[0].cd_rate as u16 + self.0[1].cd_rate as u16) / 2) as u8,
            // add -> clockwise direction is dominant
            cd_dir: self.0[0].cd_dir.saturating_add(self.0[1].cd_dir),
            // add -> saturated colors are dominant
            sat: self.0[0].sat.saturating_add(self.0[1].sat),
            // inverse add -> slower hue cycling is dominant
            hue_ratedir: (2 + (14 - self.0[0].hue_ratedir.saturating_add(self.0[1].hue_ratedir).min(14)))
                % 14,
            // min -> wider color range is dominant
            hue_base: self.0[0].hue_base.min(self.0[1].hue_base),
            // max -> wider color range is dominant
            hue_bound: self.0[0].hue_bound.max(self.0[1].hue_bound),
            // chaser -> large chaser values (which is no chaser) is dominant
            chaser: self.0[0].chaser.saturating_add(self.0[1].chaser),
            // nonlin -> brightness correction is dominant
            nonlin: self.0[0].chaser.saturating_add(self.0[1].nonlin),
        };
        // ensure that the bound is always bigger than the base
        e.hue_bound = e.hue_bound.max(e.hue_base);
        e
    }
}

pub struct Lightgenes {
    bio_ss: Bio,
    bio_pin: u5,
    _led_count: u8,
    // handles have to be kept around or else the underlying CSR is dropped
    _tx_handle: CoreHandle,
    // the CoreCsr is a convenience object that manages the CSR view of the handle
    tx: CoreCsr,
    // tracks the resources used by the object
    resource_grant: ResourceGrant,
    pub gene: Diploid,
}

impl Resources for Lightgenes {
    fn resource_spec() -> ResourceSpec {
        ResourceSpec {
            claimer: "Lightgenes".to_string(),
            cores: vec![CoreRequirement::Any],
            fifos: vec![Fifo::Fifo1],
            static_pins: vec![],
            dynamic_pin_count: 1,
        }
    }
}

impl Drop for Lightgenes {
    fn drop(&mut self) {
        for &core in self.resource_grant.cores.iter() {
            self.bio_ss.de_init_core(core).unwrap();
        }
        self.bio_ss.release_dynamic_pin(self.bio_pin.as_u8(), &Lightgenes::resource_spec().claimer).unwrap();
        self.bio_ss.release_resources(self.resource_grant.grant_id).unwrap();
    }
}

impl Lightgenes {
    pub fn new(bio_pin: u5, led_count: u8, io_mode: Option<IoConfigMode>) -> Result<Self, BioError> {
        let mut bio_ss = Bio::new();
        // claim core resource and initialize it
        let resource_grant = bio_ss.claim_resources(&Self::resource_spec())?;
        let config = CoreConfig { clock_mode: bao1x_api::bio::ClockMode::TargetFreqInt(6_666_667) };
        bio_ss.init_core(resource_grant.cores[0], lightgenes_bio_code(), config)?;
        bio_ss.set_core_run_state(&resource_grant, true);

        // claim pin resource - this only claims the resource, it does not configure it
        bio_ss.claim_dynamic_pin(bio_pin.as_u8(), &Lightgenes::resource_spec().claimer)?;
        // now configure the claimed resource
        let mut io_config = IoConfig::default();
        io_config.mapped = 1 << bio_pin.as_u32();

        // snap the outputs to the quantum of the configured core
        // don't use this - it causes ws2812 to not be compatible with other applications, e.g.
        // captouch. The main drawback is the timing is every so slightly off but it seems
        // within tolerance.
        // io_config.snap_outputs = Some(resource_grant.cores[0].into());

        io_config.mode = io_mode.unwrap_or(IoConfigMode::Overwrite);
        bio_ss.setup_io_config(io_config).unwrap();

        // safety: fifo1 is stored in this object so they aren't Drop'd before the object is
        // destroyed
        let tx_handle = unsafe { bio_ss.get_core_handle(Fifo::Fifo1) }?.expect("Didn't get FIFO1 handle");

        let mut tx = CoreCsr::from_handle(&tx_handle);
        // set FIFO1 event trigger level, so that the event triggers if there is more than
        // 0 items in the FIFO. The BIO core will use this to know if there are values waiting
        // for it in FIFO1.
        bio_ss
            .setup_fifo_event_triggers(FifoEventConfig {
                which: Fifo::Fifo1,
                trigger_slot: TriggerSlot::new_with_raw_value(0),
                level: FifoLevel::new_with_raw_value(1),
                trigger_less_than: false,
                trigger_greater_than: true,
                trigger_equal_to: true,
            })
            .expect("couldn't set FIFO trigger configuration");

        let gene = Diploid([Haploid::from_rand(), Haploid::from_rand()]);
        let phenotype = gene.phenotype();
        let ser = phenotype.serialize();

        tx.csr.wo(utralib::utra::bio_bdma::SFR_TXF1, bio_pin.as_u32());
        tx.csr.wo(utralib::utra::bio_bdma::SFR_TXF1, led_count as u32);

        for (i, s) in ser.iter().enumerate() {
            let val = (i as u32) << 8 | *s as u32;
            tx.csr.wo(utralib::utra::bio_bdma::SFR_TXF1, val);
            while tx.csr.rf(utralib::utra::bio_bdma::SFR_FLEVEL_PCLK_REGFIFO_LEVEL1) > 7 {
                // don't overflow the fifo
            }
        }

        log::debug!(
            "status: {:x} level: {}",
            tx.csr.r(utralib::utra::bio_bdma::SFR_EVENT_STATUS),
            tx.csr.r(utralib::utra::bio_bdma::SFR_FLEVEL)
        );

        Ok(Self {
            bio_ss,
            bio_pin,
            _led_count: led_count,
            tx: CoreCsr::from_handle(&tx_handle),
            // safety: tx and rx are wrapped in CSR objects whose lifetime matches that of the handles
            _tx_handle: tx_handle,
            resource_grant,
            gene,
        })
    }

    /// generates a Haploid gamete by randomly selecting portions of the Diploid genome
    pub fn meiosis(&mut self) -> Haploid {
        let mut gamete = Haploid::default();
        let parent: usize = rand::thread_rng().gen_range(0..2);
        gamete.cd_period = self.gene.0[parent].cd_period;
        gamete.cd_rate = self.gene.0[parent].cd_rate;
        gamete.cd_dir = self.gene.0[parent].cd_dir;

        gamete.sat = self.gene.0[rand::thread_rng().gen_range(0..2)].sat;

        let parent: usize = rand::thread_rng().gen_range(0..2);
        gamete.hue_ratedir = self.gene.0[parent].hue_ratedir;
        gamete.hue_base = self.gene.0[parent].hue_base;
        gamete.hue_bound = self.gene.0[parent].hue_bound;

        gamete.chaser = self.gene.0[rand::thread_rng().gen_range(0..2)].chaser;
        gamete.nonlin = self.gene.0[rand::thread_rng().gen_range(0..2)].nonlin;
        gamete
    }

    /// Test routine that causes the gene to breed against a new randomly selected "mate"
    pub fn autogamy(&mut self) {
        let mut egg = self.meiosis();
        let sperm = Haploid::from_rand();
        mutate(&mut egg, 1);
        // sperm is random, no need to mutate
        self.gene.0 = [egg, sperm];
    }

    pub fn syngamy(&mut self, mut sperm: Haploid, mut_rate: u8) {
        let mut egg = self.meiosis();
        mutate(&mut egg, mut_rate);
        mutate(&mut sperm, mut_rate);
        self.gene.0 = [egg, sperm];
    }

    /// Forces a given gene to be expressed. Does not affect the light gene state.
    pub fn force(&mut self, phenotype: Haploid) {
        log::info!("forcing {:?}", phenotype);
        let mrnas = phenotype.serialize();
        for (index, mrna) in mrnas.iter().enumerate() {
            let codon = (index as u32) << 8 | *mrna as u32;
            self.tx.csr.wo(utralib::utra::bio_bdma::SFR_TXF1, codon);
            while self.tx.csr.rf(utralib::utra::bio_bdma::SFR_FLEVEL_PCLK_REGFIFO_LEVEL1) > 7 {
                // don't overflow the fifo
            }
        }
    }

    /// Express the current phenotype by sending it to the light rendering engine
    pub fn express(&mut self) {
        let phenotype = self.gene.phenotype();
        log::info!("phenotype: {:?}", phenotype);
        let mrnas = phenotype.serialize();
        for (index, mrna) in mrnas.iter().enumerate() {
            let codon = (index as u32) << 8 | *mrna as u32;
            self.tx.csr.wo(utralib::utra::bio_bdma::SFR_TXF1, codon);
            while self.tx.csr.rf(utralib::utra::bio_bdma::SFR_FLEVEL_PCLK_REGFIFO_LEVEL1) > 7 {
                // don't overflow the fifo
            }
        }
    }

    #[allow(dead_code)]
    pub fn map(x: i32, in_min: i32, in_max: i32, out_min: i32, out_max: i32) -> Option<i32> {
        if in_max == in_min {
            return None;
        }
        Some((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    }
}

pub fn mutate(gamete: &mut Haploid, rate: u8) {
    let bits: u8 = if rate < 128 {
        1
    } else if rate < 245 {
        3
    } else {
        7
    };

    if rand::thread_rng().gen::<u8>() < rate {
        gamete.cd_period = mutation_func(gamete.cd_period, bits) % 6;
    }
    if rand::thread_rng().gen::<u8>() < rate {
        gamete.cd_rate = mutation_func(gamete.cd_rate, bits);
    }
    if rand::thread_rng().gen::<u8>() < rate {
        gamete.cd_dir = mutation_func(gamete.cd_dir, bits);
    }
    if rand::thread_rng().gen::<u8>() < rate {
        gamete.sat = mutation_func(gamete.sat, bits);
    }
    if rand::thread_rng().gen::<u8>() < rate {
        gamete.hue_ratedir = mutation_func(gamete.hue_ratedir, bits);
    }
    if rand::thread_rng().gen::<u8>() < rate {
        gamete.hue_base = mutation_func(gamete.hue_base, bits);
    }
    if rand::thread_rng().gen::<u8>() < rate {
        gamete.hue_bound = mutation_func(gamete.hue_bound, bits);
    }
    if rand::thread_rng().gen::<u8>() < rate {
        gamete.chaser = mutation_func(gamete.chaser, bits);
    }
    if rand::thread_rng().gen::<u8>() < rate {
        gamete.nonlin = mutation_func(gamete.nonlin, bits);
    }
}

fn mutation_func(gene: u8, bits: u8) -> u8 {
    gray_decode(gray_encode(gene) ^ (bits << (rand::thread_rng().gen_range(0..=7))))
}

fn gray_encode(n: u8) -> u8 { n ^ (n >> 1) }

fn gray_decode(mut n: u8) -> u8 {
    let mut p = n;
    while n >> 1 != 0 {
        n >>= 1;
        p ^= n;
    }
    p
}
