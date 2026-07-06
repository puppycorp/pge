use std::fmt;
use std::path::{Path, PathBuf};
use std::process::Command;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PngSequence {
    pub directory: PathBuf,
    pub pattern: String,
    pub start_index: u32,
    pub frame_count: u32,
}

impl PngSequence {
    pub fn new(directory: impl Into<PathBuf>, frame_count: u32) -> Self {
        Self {
            directory: directory.into(),
            pattern: "frame-%05d.png".to_string(),
            start_index: 0,
            frame_count,
        }
    }

    fn location(&self) -> PathBuf {
        self.directory.join(&self.pattern)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawRgbaSequence {
    pub directory: PathBuf,
    pub pattern: String,
    pub start_index: u32,
    pub frame_count: u32,
    pub width: u32,
    pub height: u32,
}

impl RawRgbaSequence {
    pub fn new(directory: impl Into<PathBuf>, frame_count: u32, width: u32, height: u32) -> Self {
        Self {
            directory: directory.into(),
            pattern: "frame-%05d.rgba".to_string(),
            start_index: 0,
            frame_count,
            width,
            height,
        }
    }

    fn location(&self) -> PathBuf {
        self.directory.join(&self.pattern)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Mp4EncodeRequest {
    pub input: PngSequence,
    pub output: PathBuf,
    pub fps: u32,
    pub bitrate: u32,
}

impl Mp4EncodeRequest {
    pub fn png_sequence(
        directory: impl Into<PathBuf>,
        frame_count: u32,
        fps: u32,
        output: impl Into<PathBuf>,
    ) -> Self {
        Self {
            input: PngSequence::new(directory, frame_count),
            output: output.into(),
            fps,
            bitrate: 4_000_000,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawRgbaMp4EncodeRequest {
    pub input: RawRgbaSequence,
    pub output: PathBuf,
    pub fps: u32,
    pub bitrate: u32,
}

impl RawRgbaMp4EncodeRequest {
    pub fn raw_rgba_sequence(
        directory: impl Into<PathBuf>,
        frame_count: u32,
        width: u32,
        height: u32,
        fps: u32,
        output: impl Into<PathBuf>,
    ) -> Self {
        Self {
            input: RawRgbaSequence::new(directory, frame_count, width, height),
            output: output.into(),
            fps,
            bitrate: 4_000_000,
        }
    }
}

#[derive(Debug)]
pub enum VideoError {
    EmptySequence,
    CreateOutputDirectory {
        path: PathBuf,
        source: std::io::Error,
    },
    Launch {
        source: std::io::Error,
    },
    EncoderFailed {
        status: std::process::ExitStatus,
    },
}

impl fmt::Display for VideoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptySequence => f.write_str("cannot encode an empty PNG sequence"),
            Self::CreateOutputDirectory { path, source } => {
                write!(f, "create output directory {}: {source}", path.display())
            }
            Self::Launch { source } => write!(f, "launch gst-launch-1.0: {source}"),
            Self::EncoderFailed { status } => write!(f, "gst-launch-1.0 failed with {status}"),
        }
    }
}

impl std::error::Error for VideoError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::CreateOutputDirectory { source, .. } => Some(source),
            Self::Launch { source } => Some(source),
            Self::EmptySequence | Self::EncoderFailed { .. } => None,
        }
    }
}

pub fn encode_png_sequence_to_mp4(request: &Mp4EncodeRequest) -> Result<(), VideoError> {
    if request.input.frame_count == 0 {
        return Err(VideoError::EmptySequence);
    }
    if let Some(parent) = request.output.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent).map_err(|source| {
                VideoError::CreateOutputDirectory {
                    path: parent.to_path_buf(),
                    source,
                }
            })?;
        }
    }

    let fps = request.fps.max(1);
    let status = Command::new("gst-launch-1.0")
        .arg("-q")
        .arg("multifilesrc")
        .arg(format!("location={}", request.input.location().display()))
        .arg(format!("start-index={}", request.input.start_index))
        .arg(format!(
            "stop-index={}",
            request
                .input
                .start_index
                .saturating_add(request.input.frame_count.saturating_sub(1))
        ))
        .arg(format!("num-buffers={}", request.input.frame_count))
        .arg(format!("caps=image/png,framerate={fps}/1"))
        .arg("!")
        .arg("pngdec")
        .arg("!")
        .arg("videoconvert")
        .arg("!")
        .arg(format!("video/x-raw,format=I420,framerate={fps}/1"))
        .arg("!")
        .arg("openh264enc")
        .arg(format!("bitrate={}", request.bitrate))
        .arg(format!("gop-size={fps}"))
        .arg("!")
        .arg("h264parse")
        .arg("config-interval=-1")
        .arg("!")
        .arg("video/x-h264,stream-format=avc,alignment=au")
        .arg("!")
        .arg("mp4mux")
        .arg("faststart=true")
        .arg("!")
        .arg("filesink")
        .arg(format!("location={}", request.output.display()))
        .status()
        .map_err(|source| VideoError::Launch { source })?;

    if !status.success() {
        return Err(VideoError::EncoderFailed { status });
    }
    Ok(())
}

pub fn encode_raw_rgba_sequence_to_mp4(
    request: &RawRgbaMp4EncodeRequest,
) -> Result<(), VideoError> {
    if request.input.frame_count == 0 {
        return Err(VideoError::EmptySequence);
    }
    if let Some(parent) = request.output.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent).map_err(|source| {
                VideoError::CreateOutputDirectory {
                    path: parent.to_path_buf(),
                    source,
                }
            })?;
        }
    }

    let fps = request.fps.max(1);
    let status = Command::new("gst-launch-1.0")
        .arg("-q")
        .arg("multifilesrc")
        .arg(format!("location={}", request.input.location().display()))
        .arg(format!("start-index={}", request.input.start_index))
        .arg(format!(
            "stop-index={}",
            request
                .input
                .start_index
                .saturating_add(request.input.frame_count.saturating_sub(1))
        ))
        .arg(format!("num-buffers={}", request.input.frame_count))
        .arg("!")
        .arg("rawvideoparse")
        .arg("format=rgba")
        .arg(format!("width={}", request.input.width))
        .arg(format!("height={}", request.input.height))
        .arg(format!("framerate={fps}/1"))
        .arg("!")
        .arg("videoconvert")
        .arg("!")
        .arg(format!("video/x-raw,format=I420,framerate={fps}/1"))
        .arg("!")
        .arg("openh264enc")
        .arg(format!("bitrate={}", request.bitrate))
        .arg(format!("gop-size={fps}"))
        .arg("!")
        .arg("h264parse")
        .arg("config-interval=-1")
        .arg("!")
        .arg("video/x-h264,stream-format=avc,alignment=au")
        .arg("!")
        .arg("mp4mux")
        .arg("faststart=true")
        .arg("!")
        .arg("filesink")
        .arg(format!("location={}", request.output.display()))
        .status()
        .map_err(|source| VideoError::Launch { source })?;

    if !status.success() {
        return Err(VideoError::EncoderFailed { status });
    }
    Ok(())
}

pub fn default_frame_path(directory: &Path, index: u32) -> PathBuf {
    directory.join(format!("frame-{index:05}.png"))
}

pub fn default_raw_rgba_frame_path(directory: &Path, index: u32) -> PathBuf {
    directory.join(format!("frame-{index:05}.rgba"))
}
