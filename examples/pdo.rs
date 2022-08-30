use std::{
    collections::HashMap,
    convert::TryInto,
    env,
    fs::File,
    io::{self, Read},
    thread,
    time::{Duration, SystemTime}, f32::consts::PI,
};

use ethercat::{
    AlState, DomainIdx, Idx, Master, MasterAccess, Offset, PdoCfg, PdoEntryIdx, PdoEntryInfo,
    PdoEntryPos, PdoIdx, SlaveAddr, SlaveId, SlavePos, SmCfg, SubIdx,  
};
use ethercat_esi::EtherCatInfo;

type BitLen = u8;


const SLAVE_ID: usize = 1;
const OFFSET: usize = SLAVE_ID * 28;


#[derive(Debug)]
struct DataFrame {
    control_word: Vec<u8>,
    mode_of_operation: u8,
    target_position: u32,
    velocity_offset: u32,
    target_torque: u16,

    status_word: Vec<u8>,
    mode_of_operation_display: u8,
    position_actual_value: u32,
    velocity_actual_value: u32,
    torque_actual_value: u16,
    error_code: u16,
}

impl DataFrame {
    pub fn from_data(data: &[u8]) -> Self {
        DataFrame {
            control_word: get_bits_set(u16::from_le_bytes(data[OFFSET..OFFSET + 2].try_into().unwrap())),
            mode_of_operation: data[OFFSET + 2],
            target_position: u32::from_le_bytes(data[OFFSET + 3..OFFSET + 7].try_into().unwrap()),
            velocity_offset: u32::from_le_bytes(data[OFFSET + 7..OFFSET + 11].try_into().unwrap()),
            target_torque: u16::from_le_bytes(data[OFFSET + 11..OFFSET + 13].try_into().unwrap()),

            status_word: get_bits_set(u16::from_le_bytes(data[OFFSET + 13..OFFSET + 15].try_into().unwrap())),
            mode_of_operation_display: data[OFFSET + 15],
            position_actual_value: u32::from_le_bytes(data[OFFSET + 16..OFFSET + 20].try_into().unwrap()),
            velocity_actual_value: u32::from_le_bytes(data[OFFSET + 20..OFFSET + 24].try_into().unwrap()),
            torque_actual_value: u16::from_le_bytes(data[OFFSET + 24..OFFSET + 26].try_into().unwrap()),
            error_code: u16::from_le_bytes(data[OFFSET + 26..OFFSET + 28].try_into().unwrap()),
        }
    }
}

enum State {
    A, B, C, D, E, F, G,
}




pub fn main() -> Result<(), io::Error> {
    env_logger::init();

    let args: Vec<_> = env::args().collect();
    let file_name = match args.len() {
        2 => &args[1],
        _ => {
            println!("usage: {} ESI-FILE", env!("CARGO_PKG_NAME"));
            return Ok(());
        }
    };

    log::debug!("Parse XML file \"{}\"", file_name);
    let mut esi_file = File::open(file_name)?;
    let mut esi_xml_string = String::new();
    esi_file.read_to_string(&mut esi_xml_string)?;
    let esi = EtherCatInfo::from_xml_str(&esi_xml_string)?;

    let (mut master, domain_idx, offsets) = init_master(&esi, 0_u32)?;

    let cycle_time = Duration::from_millis(1);
    // let cycle_time = Duration::from_micros(100);
    master.activate()?;

    for (s, o) in &offsets {
        log::info!("PDO offsets of Slave {}:", u16::from(*s));
        for (name, (pdo, bit_len, offset)) in o {
            log::info!(
                " - \"{}\" : {:X}:{:X} - {:?}, bit length: {}",
                name,
                u16::from(pdo.idx),
                u8::from(pdo.sub_idx),
                offset,
                bit_len
            );
        }
    }

    let mut state = State::A;
    let mut t0 = SystemTime::now();

    let mut t = SystemTime::now();

    let mut i = 0;
    
    loop {
        master.receive()?;
        master.domain(domain_idx).process()?;
        master.domain(domain_idx).queue()?;

        let data = master.domain_data(domain_idx)?;

        let df = DataFrame::from_data(data);
        // log::debug!("{:?}", data);

        match state {
            State::A => {
                let dt = t0.elapsed().unwrap();
                if dt.as_secs_f32() > 2.0 {
                    state = State::B;
                    t0 = SystemTime::now();
                }
            }
            State::B => {
                data[OFFSET + 2] = 0x01; // Modes of operation 0x01 (Profile Position Mode)
                let dt = t0.elapsed().unwrap();
                if dt.as_millis() > 1 {
                    state = State::C;
                    t0 = SystemTime::now();
                }
            },
            State::C => {
                data[OFFSET + 0] = 0x06; // Controlword (shutdown)
                let dt = t0.elapsed().unwrap();
                if dt.as_millis() > 1 {
                    state = State::D;
                    t0 = SystemTime::now();
                }            
            },
            State::D => {
                data[OFFSET + 0] = 0x07; // Controlword (Switch On)
                let dt = t0.elapsed().unwrap();
                if dt.as_millis() > 1 {
                    state = State::E;
                    t0 = SystemTime::now();
                }            
            },
            State::E => {
                data[OFFSET + 0] = 0x0F; // Controlword (Switch On & Enable)
                let dt = t0.elapsed().unwrap();
                if dt.as_millis() > 1 {
                    state = State::F;
                    t0 = SystemTime::now();
                }                
            },
            State::F => {
                data[OFFSET + 0] = 0x3F; // Controlword (Absolute pos, start immediatly)

                let f = 0.5_f32;
                let amp = 2000.0_f32;

                let dt = t.elapsed().unwrap().as_secs_f32();
                let x = amp * (2.0 * PI * f * dt).sin();
                let x = x as i32 + 5000;

                let pos = x as u32;

                let bytes = pos.to_le_bytes();
                data[OFFSET + 3] = bytes[0];
                data[OFFSET + 4] = bytes[1];
                data[OFFSET + 5] = bytes[2];
                data[OFFSET + 6] = bytes[3];
                let dt = t0.elapsed().unwrap();
                if dt.as_millis() >= 1 {
                    state = State::G;
                    t0 = SystemTime::now();
                }    
            },
            State::G => {
                data[OFFSET + 0] = 0x0F; // Controlword (Absolute pos, start immediatly)
                let dt = t0.elapsed().unwrap();
                if dt.as_millis() >= 1 {
                    state = State::F;
                    t0 = SystemTime::now();
                }  

                let error = df.target_position as i32 - df.position_actual_value as i32;
                log::debug!("{}", error);
            },

            _ => {},
        }

        master.send()?;

        thread::sleep(cycle_time);
    }
}

fn init_master(
    esi: &EtherCatInfo,
    idx: u32,
) -> Result<
    (
        Master,
        DomainIdx,
        HashMap<SlavePos, HashMap<String, (PdoEntryIdx, BitLen, Offset)>>,
    ),
    io::Error,
> {
    let mut master = Master::open(idx, MasterAccess::ReadWrite)?;
    master.reserve()?;

    let domain_idx = master.create_domain()?;

    let mut offsets: HashMap<SlavePos, HashMap<String, (PdoEntryIdx, u8, Offset)>> = HashMap::new();

    for (dev_nr, dev) in esi.description.devices.iter().enumerate() {
        let slave_pos = SlavePos::from(dev_nr as u16);
        log::debug!("Request PreOp state for {:?}", slave_pos);
        master.request_state(slave_pos, AlState::PreOp)?;
        let slave_info = master.get_slave_info(slave_pos)?;
        log::info!("Found device {}:{:?}", dev.name, slave_info);

        let slave_addr = SlaveAddr::ByPos(dev_nr as u16);
        let slave_id = SlaveId {
            vendor_id: esi.vendor.id,
            product_code: dev.product_code,
        };
        let mut config = master.configure_slave(slave_addr, slave_id)?;
        let mut entry_offsets: HashMap<String, (PdoEntryIdx, u8, Offset)> = HashMap::new();

        let rx_pdos: Vec<PdoCfg> = dev
            .rx_pdo
            .iter()
            .map(|pdo| PdoCfg {
                idx: PdoIdx::from(pdo.index),
                entries: pdo
                    .entries
                    .iter()
                    .enumerate()
                    .map(|(i, e)| PdoEntryInfo {
                        entry_idx: PdoEntryIdx {
                            idx: Idx::from(e.index),
                            sub_idx: SubIdx::from(e.sub_index.unwrap_or(1) as u8),
                        },
                        bit_len: e.bit_len as u8,
                        name: e.name.clone().unwrap_or(String::new()),
                        pos: PdoEntryPos::from(i as u8),
                    })
                    .collect(),
            })
            .collect();

        let tx_pdos: Vec<PdoCfg> = dev
            .tx_pdo
            .iter()
            .map(|pdo| PdoCfg {
                idx: PdoIdx::from(pdo.index),
                entries: pdo
                    .entries
                    .iter()
                    .enumerate()
                    .map(|(i, e)| PdoEntryInfo {
                        entry_idx: PdoEntryIdx {
                            idx: Idx::from(e.index),
                            sub_idx: SubIdx::from(e.sub_index.unwrap_or(1) as u8),
                        },
                        bit_len: e.bit_len as u8,
                        name: e.name.clone().unwrap_or(String::new()),
                        pos: PdoEntryPos::from(i as u8),
                    })
                    .collect(),
            })
            .collect();

        let output = SmCfg::output(2.into());
        let input = SmCfg::input(3.into());

        for pdo in &rx_pdos {
            // Positions of RX PDO
            log::debug!("Positions of RX PDO 0x{:X}:", u16::from(pdo.idx));
            for entry in &pdo.entries {
                log::debug!(
                    "\t PDO RX entry \"{}\": {:?} {:?}",
                    entry.name,
                    entry.pos,
                    entry.entry_idx
                );

                let offset = config.register_pdo_entry(entry.entry_idx, domain_idx)?;
                entry_offsets.insert(entry.name.clone(), (entry.entry_idx, entry.bit_len, offset));
            }
        }
        for pdo in &tx_pdos {
            // Positions of TX PDO
            log::debug!("Positions of TX PDO 0x{:X}:", u16::from(pdo.idx));
            for entry in &pdo.entries {
                log::debug!(
                    "\t PDO TX entry \"{}\": {:?} {:?}",
                    entry.name,
                    entry.pos,
                    entry.entry_idx
                );

                let offset = config.register_pdo_entry(entry.entry_idx, domain_idx)?;
                entry_offsets.insert(entry.name.clone(), (entry.entry_idx, entry.bit_len, offset));
            }
        }

        config.config_sm_pdos(output, &rx_pdos)?;
        config.config_sm_pdos(input, &tx_pdos)?;

        let cfg_index = config.index();
        let cfg_info = master.get_config_info(cfg_index)?;
        log::info!("Config info: {:#?}", cfg_info);
        if cfg_info.slave_position.is_none() {
            return Err(io::Error::new(
                io::ErrorKind::Other,
                "Unable to configure slave",
            ));
        }
        offsets.insert(slave_pos, entry_offsets);
    }
    Ok((master, domain_idx, offsets))
}


fn get_bits_set(input: u16) -> Vec<u8> {
    let mut v = Vec::new();

    for b in 0..16 {
        if get_bit_at(input, b) {
            v.push(1);
        }
        else {
            v.push(0);
        }
    }
    v
}

fn get_bit_at(input: u16, n: u8) -> bool {
    assert!(n < 16);
    input & (1 << n) != 0
}