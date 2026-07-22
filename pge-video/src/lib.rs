use std::fmt;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, Command, Stdio};

use pge_core::WorldState;
use pge_renderer::{OffscreenRenderer, RenderError, RenderRequest};

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
    WriteFrame {
        path: PathBuf,
        source: std::io::Error,
    },
    StreamWrite {
        source: std::io::Error,
    },
    Render {
        source: RenderError,
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
            Self::WriteFrame { path, source } => {
                write!(f, "write video frame {}: {source}", path.display())
            }
            Self::StreamWrite { source } => write!(f, "write streaming video frame: {source}"),
            Self::Render { source } => write!(f, "render video frame: {source}"),
            Self::Launch { source } => write!(f, "launch gst-launch-1.0: {source}"),
            Self::EncoderFailed { status } => write!(f, "gst-launch-1.0 failed with {status}"),
        }
    }
}

impl std::error::Error for VideoError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::CreateOutputDirectory { source, .. } => Some(source),
            Self::WriteFrame { source, .. } => Some(source),
            Self::StreamWrite { source } => Some(source),
            Self::Render { source } => Some(source),
            Self::Launch { source } => Some(source),
            Self::EmptySequence | Self::EncoderFailed { .. } => None,
        }
    }
}

/// H.264 MP4 encoder that accepts raw RGBA frames through a persistent stdin.
///
/// The encoder writes to a sibling temporary file and publishes the final MP4
/// only after `finish` closes stdin and GStreamer has finalized the container.
pub struct StreamingRgbaMp4Encoder {
    child: Child,
    stdin: Option<ChildStdin>,
    output: PathBuf,
    temporary_output: PathBuf,
    frame_bytes: usize,
    frame_count: u64,
}

impl StreamingRgbaMp4Encoder {
    pub fn start(
        output: impl Into<PathBuf>,
        width: u32,
        height: u32,
        fps: u32,
        bitrate: u32,
    ) -> Result<Self, VideoError> {
        let output = output.into();
        if let Some(parent) = output.parent() {
            if !parent.as_os_str().is_empty() {
                std::fs::create_dir_all(parent).map_err(|source| {
                    VideoError::CreateOutputDirectory {
                        path: parent.to_path_buf(),
                        source,
                    }
                })?;
            }
        }
        let temporary_output = output.with_extension("mp4.partial");
        let fps = fps.max(1);
        let frame_bytes = width as usize * height as usize * 4;
        let mut child = Command::new("gst-launch-1.0")
            .arg("-q")
            .arg("fdsrc")
            .arg("fd=0")
            .arg("!")
            .arg("rawvideoparse")
            .arg("format=rgba")
            .arg(format!("width={width}"))
            .arg(format!("height={height}"))
            .arg(format!("framerate={fps}/1"))
            .arg("!")
            .arg("videoconvert")
            .arg("!")
            .arg(format!("video/x-raw,format=I420,framerate={fps}/1"))
            .arg("!")
            .arg("openh264enc")
            .arg(format!("bitrate={bitrate}"))
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
            .arg(format!("location={}", temporary_output.display()))
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .map_err(|source| VideoError::Launch { source })?;
        let stdin = child.stdin.take().ok_or_else(|| VideoError::Launch {
            source: std::io::Error::other("gst-launch-1.0 stdin was not piped"),
        })?;
        Ok(Self {
            child,
            stdin: Some(stdin),
            output,
            temporary_output,
            frame_bytes,
            frame_count: 0,
        })
    }

    pub fn push_rgba_frame(&mut self, bytes: &[u8]) -> Result<(), VideoError> {
        if bytes.len() != self.frame_bytes {
            return Err(VideoError::StreamWrite {
                source: std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!(
                        "expected {} RGBA bytes, got {}",
                        self.frame_bytes,
                        bytes.len()
                    ),
                ),
            });
        }
        self.stdin
            .as_mut()
            .ok_or_else(|| VideoError::StreamWrite {
                source: std::io::Error::new(std::io::ErrorKind::BrokenPipe, "encoder is finalized"),
            })?
            .write_all(bytes)
            .map_err(|source| VideoError::StreamWrite { source })?;
        self.frame_count = self.frame_count.saturating_add(1);
        Ok(())
    }

    pub fn finish(mut self) -> Result<VideoRecordOutput, VideoError> {
        drop(self.stdin.take());
        let status = self
            .child
            .wait()
            .map_err(|source| VideoError::Launch { source })?;
        if !status.success() {
            return Err(VideoError::EncoderFailed { status });
        }
        std::fs::rename(&self.temporary_output, &self.output).map_err(|source| {
            VideoError::WriteFrame {
                path: self.output.clone(),
                source,
            }
        })?;
        Ok(VideoRecordOutput {
            output: self.output,
            frame_count: self.frame_count.min(u64::from(u32::MAX)) as u32,
        })
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct VideoRecordRequest {
    pub output: PathBuf,
    pub fps: u32,
    pub frame_count: u32,
    pub work_dir: PathBuf,
    pub bitrate: u32,
}

impl VideoRecordRequest {
    pub fn mp4(
        output: impl Into<PathBuf>,
        fps: u32,
        frame_count: u32,
        work_dir: impl Into<PathBuf>,
    ) -> Self {
        Self {
            output: output.into(),
            fps,
            frame_count,
            work_dir: work_dir.into(),
            bitrate: 4_000_000,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VideoRecordOutput {
    pub output: PathBuf,
    pub frame_count: u32,
}

#[derive(Clone, Debug, Default)]
pub struct VideoRecorder;

impl VideoRecorder {
    pub fn record<R, F>(
        &self,
        renderer: &mut R,
        world: &mut WorldState,
        render_request: &RenderRequest,
        request: &VideoRecordRequest,
        mut update: F,
    ) -> Result<VideoRecordOutput, VideoError>
    where
        R: OffscreenRenderer,
        F: FnMut(u32, &mut WorldState),
    {
        if request.frame_count == 0 {
            return Err(VideoError::EmptySequence);
        }
        std::fs::create_dir_all(&request.work_dir).map_err(|source| {
            VideoError::CreateOutputDirectory {
                path: request.work_dir.clone(),
                source,
            }
        })?;

        let resolution = render_request.resolution;
        for index in 0..request.frame_count {
            update(index, world);
            let frame = renderer
                .render_rgba(world, render_request)
                .map_err(|source| VideoError::Render { source })?;
            let path = request.work_dir.join(format!("frame-{index:05}.rgba"));
            let mut file =
                std::fs::File::create(&path).map_err(|source| VideoError::WriteFrame {
                    path: path.clone(),
                    source,
                })?;
            file.write_all(&frame.bytes)
                .map_err(|source| VideoError::WriteFrame {
                    path: path.clone(),
                    source,
                })?;
        }

        let mut encode_request = RawRgbaMp4EncodeRequest::raw_rgba_sequence(
            request.work_dir.clone(),
            request.frame_count,
            resolution[0],
            resolution[1],
            request.fps,
            request.output.clone(),
        );
        encode_request.bitrate = request.bitrate;
        encode_raw_rgba_sequence_to_mp4(&encode_request)?;
        Ok(VideoRecordOutput {
            output: request.output.clone(),
            frame_count: request.frame_count,
        })
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
