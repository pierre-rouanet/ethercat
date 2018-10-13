use std::io;
use crate::ec;

pub type Error = io::Error;
pub type Result<T> = io::Result<T>;

pub type MasterIndex = u32;
pub type DomainIndex = u32;
pub type SlaveConfigIndex = u32;

#[derive(Clone, Copy)]
pub struct DomainHandle(pub(crate) usize);


/// An EtherCAT slave identification, consisting of vendor ID and product code.
#[derive(Debug, Clone, Copy)]
pub struct SlaveId {
    pub vendor_id: u32,
    pub product_code: u32,
}

impl SlaveId {
    #![allow(non_snake_case)]

    pub fn EL(nr: u32) -> SlaveId {
        SlaveId { vendor_id: 0x2, product_code: (nr << 16) | 0x3052 }
    }

    pub fn EK(nr: u32) -> SlaveId {
        SlaveId { vendor_id: 0x2, product_code: (nr << 16) | 0x2c52 }
    }
}


/// An EtherCAT slave revision identification.
#[derive(Debug, Clone, Copy)]
pub struct SlaveRev {
    pub revision_number: u32,
    pub serial_number: u32,
}


/// An EtherCAT slave, which is specified either by absolute position in the
/// ring or by offset from a given alias.
#[derive(Clone, Copy)]
pub enum SlaveAddr {
    ByPos(u16),
    ByAlias(u16, u16)
}

impl SlaveAddr {
    pub(crate) fn as_pair(&self) -> (u16, u16) {
        match *self {
            SlaveAddr::ByPos(x) => (0, x),
            SlaveAddr::ByAlias(x, y) => (x, y),
        }
    }
}

/// Offset of a PDO entry in the domain image.
#[derive(Debug, PartialEq, Eq)]
pub struct Offset {
    pub byte: usize,
    pub bit: u32,
}

#[derive(Debug)]
pub struct MasterInfo {
    pub slave_count: u32,
    pub link_up: bool,
    pub scan_busy: bool,
    pub app_time: u64,
}

#[derive(Debug)]
pub struct MasterState {
    pub slaves_responding: u32,
    pub al_states: u8,
    pub link_up: bool,
}

#[derive(Debug)]
pub struct SlaveInfo {
    pub name: String,
    pub ring_pos: u16,
    pub id: SlaveId,
    pub rev: SlaveRev,
    pub alias: u16,
    pub current_on_ebus: i16,
    pub al_state: AlState,
    pub error_flag: u8,
    pub sync_count: u8,
    pub sdo_count: u16,
    pub ports: [SlavePortInfo; ec::EC_MAX_PORTS as usize],
}

#[derive(Debug, Clone, Copy)]
pub enum SlavePortType {
    NotImplemented,
    NotConfigured,
    EBus,
    MII,
}

impl Default for SlavePortType {
    fn default() -> Self {
        SlavePortType::NotImplemented
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct SlavePortLink {
    pub link_up: bool,
    pub loop_closed: bool,
    pub signal_detected: bool,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct SlavePortInfo {
    pub desc: SlavePortType,
    pub link: SlavePortLink,
    pub receive_time: u32,
    pub next_slave: u16,
    pub delay_to_next_dc: u32,
}

#[derive(Debug)]
pub struct SlaveConfigState {
    pub online: bool,
    pub operational: bool,
    pub al_state: AlState,
}

#[derive(Clone, Copy)]
pub enum SyncDirection {
    Invalid,
    Output,
    Input,
}

#[derive(Clone, Copy)]
pub enum WatchdogMode {
    Default,
    Enable,
    Disable,
}

pub type SmIndex = u8;
pub type PdoIndex = u16;

#[derive(Clone, Copy)]
pub struct PdoEntryIndex {
    pub index: u16,
    pub subindex: u8,
}

#[derive(Clone, Copy)]
pub struct SdoIndex {
    pub index: u16,
    pub subindex: u8,
}

pub struct SyncInfo<'a> {
    pub index: SmIndex,
    pub direction: SyncDirection,
    pub watchdog_mode: WatchdogMode,
    pub pdos: &'a [PdoInfo<'a>],
}

impl SyncInfo<'static> {
    pub const fn input(index: SmIndex, pdos: &'static [PdoInfo<'static>]) -> Self {
        SyncInfo { index, direction: SyncDirection::Input, watchdog_mode: WatchdogMode::Default, pdos }
    }

    pub const fn output(index: SmIndex, pdos: &'static [PdoInfo<'static>]) -> Self {
        SyncInfo { index, direction: SyncDirection::Output, watchdog_mode: WatchdogMode::Default, pdos }
    }
}

pub struct PdoInfo<'a> {
    pub index: PdoIndex,
    pub entries: &'a [PdoEntryInfo],
}

impl PdoInfo<'static> {
    pub const fn default(index: PdoIndex) -> PdoInfo<'static> {
        PdoInfo { index, entries: &[] }
    }
}

pub struct PdoEntryInfo {
    pub index: PdoEntryIndex,
    pub bit_length: u8,
}

#[derive(Debug)]
pub enum AlState {
    Init = 1,
    Preop = 2,
    Safeop = 4,
    Op = 8,
}

impl From<u32> for AlState {
    fn from(st: u32) -> Self {
        match st {
            1 => AlState::Init,
            2 => AlState::Preop,
            4 => AlState::Safeop,
            8 => AlState::Op,
            x => panic!("invalid state {}", x),
        }
    }
}

pub trait SdoData { }

impl SdoData for u8 { }
impl SdoData for u16 { }
impl SdoData for u32 { }

#[derive(Debug)]
pub struct DomainState {
    pub working_counter: u32,
    pub wc_state: WcState,
    pub redundancy_active: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum WcState {
    Zero = 0,
    Incomplete,
    Complete,
}

impl From<u32> for WcState {
    fn from(st: u32) -> Self {
        match st {
            0 => WcState::Zero,
            1 => WcState::Incomplete,
            2 => WcState::Complete,
            x => panic!("invalid state {}", x),
        }
    }
}
