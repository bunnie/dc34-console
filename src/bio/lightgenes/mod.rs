mod lightgenes;

use arbitrary_int::{Number, u5};
use bao1x_api::bio::*;
use bao1x_api::bio_resources::*;
use bao1x_hal::bio::{Bio, CoreCsr};
use dc34_api::*;
use lightgenes::*;

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
    pub gene: Option<Diploid>,
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

        tx.csr.wo(utralib::utra::bio_bdma::SFR_TXF1, bio_pin.as_u32());
        tx.csr.wo(utralib::utra::bio_bdma::SFR_TXF1, led_count as u32);

        Ok(Self {
            bio_ss,
            bio_pin,
            _led_count: led_count,
            tx: CoreCsr::from_handle(&tx_handle),
            // safety: tx and rx are wrapped in CSR objects whose lifetime matches that of the handles
            _tx_handle: tx_handle,
            resource_grant,
            gene: None,
        })
    }

    /// generates a Haploid gamete by randomly selecting portions of the Diploid genome
    pub fn meiosis(&mut self) -> Option<Haploid> {
        if let Some(gene) = self.gene.as_mut() { Some(gene.meiosis()) } else { None }
    }

    /// Test routine that causes the gene to breed against a new randomly selected "mate"
    pub fn autogamy(&mut self) {
        let sperm = Haploid::from_rand();
        let egg = self.meiosis();
        if let Some(mut egg) = egg {
            mutate(&mut egg, MutationRate::Baseline);
            // sperm is random, no need to mutate
            self.gene.replace(Diploid([egg, sperm]));
        }
    }

    pub fn syngamy(&mut self, mut sperm: Haploid, mut_rate: MutationRate) {
        let egg = self.meiosis();
        if let Some(mut egg) = egg {
            mutate(&mut egg, mut_rate);
            mutate(&mut sperm, mut_rate);
            self.gene.replace(Diploid([egg, sperm]));
        }
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
        if let Some(gene) = &self.gene {
            let phenotype = gene.phenotype();
            log::info!("phenotype: {:?}", phenotype);
            let mrnas = phenotype.serialize();
            for (index, mrna) in mrnas.iter().enumerate() {
                let codon = (index as u32) << 8 | *mrna as u32;
                self.tx.csr.wo(utralib::utra::bio_bdma::SFR_TXF1, codon);
                while self.tx.csr.rf(utralib::utra::bio_bdma::SFR_FLEVEL_PCLK_REGFIFO_LEVEL1) > 7 {
                    // don't overflow the fifo
                }
            }
            self.tx.csr.wo(utralib::utra::bio_bdma::SFR_TXF1, 0x0000_0000);
        } else {
            // do nothing, we haven't been initialized
        }
    }

    pub fn jack_eyes(&mut self, flash: bool) {
        while self.tx.csr.rf(utralib::utra::bio_bdma::SFR_FLEVEL_PCLK_REGFIFO_LEVEL1) > 7 {
            // don't overflow the fifo
        }
        if flash {
            self.tx.csr.wo(utralib::utra::bio_bdma::SFR_TXF1, 0x2000_0000);
        } else {
            self.tx.csr.wo(utralib::utra::bio_bdma::SFR_TXF1, 0x1000_0000);
        }
        self.tx.csr.wo(utralib::utra::bio_bdma::SFR_TXF1, 0x0000_0000);
    }

    /// Pause now has the behavior of automatically un-pausing. The purpose of this
    /// is to briefly stop rendering while the device goes between clock modes.
    pub fn pause_rendering(&mut self) {
        // log::info!("pause!!!");
        while self.tx.csr.rf(utralib::utra::bio_bdma::SFR_FLEVEL_PCLK_REGFIFO_LEVEL1) > 7 {
            // don't overflow the fifo
        }
        self.tx.csr.wo(utralib::utra::bio_bdma::SFR_TXF1, 0x8000_0000);
        self.tx.csr.wo(utralib::utra::bio_bdma::SFR_TXF1, 0x0000_0000);
    }

    /*
    pub fn resume_rendering(&mut self) {
        while self.tx.csr.rf(utralib::utra::bio_bdma::SFR_FLEVEL_PCLK_REGFIFO_LEVEL1) > 7 {
            // don't overflow the fifo
        }
        self.tx.csr.wo(utralib::utra::bio_bdma::SFR_TXF1, 0x4000_0000);
    }
    */

    #[allow(dead_code)]
    pub fn map(x: i32, in_min: i32, in_max: i32, out_min: i32, out_max: i32) -> Option<i32> {
        if in_max == in_min {
            return None;
        }
        Some((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    }
}
