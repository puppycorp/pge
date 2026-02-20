use colored::Colorize;
use log::Level;
use log::Metadata;
use log::Record;
use log::SetLoggerError;
use std::env;
use std::sync::OnceLock;
use std::thread;

struct SimpleLogger;

#[macro_export]
macro_rules! log1 {
    ($($arg:tt)*) => {
        if $crate::debug_level() >= 1 {
            log::info!($($arg)*);
        }
    };
}

#[macro_export]
macro_rules! log2 {
    ($($arg:tt)*) => {
        if $crate::debug_level() >= 2 {
            log::debug!($($arg)*);
        }
    };
}

#[macro_export]
macro_rules! log3 {
    ($($arg:tt)*) => {
        if $crate::debug_level() >= 3 {
            log::trace!($($arg)*);
        }
    };
}

#[macro_export]
macro_rules! log4 {
    ($($arg:tt)*) => {
        if $crate::debug_level() >= 4 {
            log::trace!($($arg)*);
        }
    };
}

impl log::Log for SimpleLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        match debug_level() {
            0 => false,
            1 => metadata.level() <= Level::Info,
            2 => {
                if metadata.level() > Level::Debug {
                    return false;
                }
                if metadata.level() == Level::Debug {
                    let target = metadata.target();
                    if target.starts_with("wgpu") {
                        return false;
                    }
                }
                true
            }
            _ => metadata.level() <= Level::Trace,
        }
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let level = match record.level() {
                Level::Error => "ERROR".red(),
                Level::Warn => "WARN".yellow(),
                Level::Info => "INFO".green(),
                Level::Debug => "DEBUG".blue(),
                Level::Trace => "TRACE".magenta(),
            };
            let thread_id = thread::current().id();
            println!("[{}] [{:?}] - {}", level, thread_id, record.args());
        }
    }

    fn flush(&self) {}
}

static LOGGER: SimpleLogger = SimpleLogger;

pub fn init_logging() {
    let level = match debug_level() {
        0 => log::LevelFilter::Off,
        1 => log::LevelFilter::Info,
        2 => log::LevelFilter::Debug,
        _ => log::LevelFilter::Trace,
    };
    log::set_logger(&LOGGER)
        .map(|()| log::set_max_level(level)).unwrap();
}

pub fn debug_level() -> u8 {
    static LEVEL: OnceLock<u8> = OnceLock::new();
    *LEVEL.get_or_init(|| {
        env::var("DEBUG")
            .ok()
            .and_then(|v| v.parse::<u8>().ok())
            .map(|v| v.min(4))
            .unwrap_or(0)
    })
}
