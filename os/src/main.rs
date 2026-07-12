use std::env;
use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::path::Path;
use anyhow::{Context, Result};
use std::time::Instant;

struct TrackingReader<R: Read> {
    inner: R,
    bytes_read: u64,
    total_bytes: u64,
    last_report: Instant,
}

impl<R: Read> Read for TrackingReader<R> {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let n = self.inner.read(buf)?;
        self.bytes_read += n as u64;
        
        let now = Instant::now();
        if now.duration_since(self.last_report).as_millis() > 500 {
            let progress = (self.bytes_read as f64 / self.total_bytes as f64) * 100.0;
            println!("PROGRESS: {:.1}", progress);
            let _ = io::stdout().flush();
            self.last_report = now;
        }
        
        Ok(n)
    }
}

fn main() -> Result<()> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("Usage: {} <image_path> <target_device>", args[0]);
        std::process::exit(1);
    }

    let image_path = Path::new(&args[1]);
    let target_dev = Path::new(&args[2]);

    println!("PROGRESS: 0.0");
    
    // 1. Open source file
    let source_file = File::open(image_path)
        .with_context(|| format!("Failed to open image file: {:?}", image_path))?;
    
    let total_size = source_file.metadata()?.len();

    let tracking_reader = TrackingReader {
        inner: source_file,
        bytes_read: 0,
        total_bytes: total_size,
        last_report: Instant::now(),
    };

    // 2. Open target device
    #[cfg(target_os = "linux")]
    {
        let dev_str = target_dev.to_string_lossy();
        let _ = std::process::Command::new("sh")
            .arg("-c")
            .arg(format!("umount {}* 2>/dev/null", dev_str))
            .status();
    }
    
    #[cfg(target_os = "macos")]
    {
        let _ = std::process::Command::new("diskutil")
            .arg("unmountDisk")
            .arg(target_dev)
            .status();
    }
    
    #[cfg(target_os = "windows")]
    {
        // target_dev from livedisk-core on Windows looks like "\\.\PhysicalDrive1"
        let dev_str = target_dev.to_string_lossy();
        if let Some(num_str) = dev_str.strip_prefix(r"\\.\PhysicalDrive") {
            if let Ok(num) = num_str.parse::<u32>() {
                let script = format!(
                    "Get-Partition -DiskNumber {} | Get-Volume | Where-Object DriveLetter -ne $null | ForEach-Object {{ mountvol \"$($_.DriveLetter):\" /d }}", 
                    num
                );
                let _ = std::process::Command::new("powershell")
                    .args(["-NoProfile", "-Command", &script])
                    .status();
            }
        }
    }

    let mut target_file = OpenOptions::new()
        .write(true)
        .open(target_dev)
        .with_context(|| format!("Failed to open target device: {:?}", target_dev))?;

    // 3. Stream data
    let is_zst = image_path.extension().and_then(|e| e.to_str()) == Some("zst");

    if is_zst {
        let mut decoder = zstd::Decoder::new(tracking_reader)
            .context("Failed to initialize zstd decoder")?;
        io::copy(&mut decoder, &mut target_file).context("Failed to write decompressed data to device")?;
    } else {
        let mut reader = tracking_reader;
        io::copy(&mut reader, &mut target_file).context("Failed to write data to device")?;
    }

    // 4. Ensure data is written
    target_file.sync_all().context("Failed to sync target device")?;

    #[cfg(target_os = "linux")]
    {
        let _ = std::process::Command::new("partprobe")
            .arg(target_dev)
            .status();
    }

    println!("PROGRESS: 100.0");
    Ok(())
}
