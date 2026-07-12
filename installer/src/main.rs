use slint::*;
use std::rc::Rc;
use std::path::{Path, PathBuf};

slint::include_modules!();

fn main() -> Result<(), slint::PlatformError> {
    let main_window = MainWindow::new()?;

    setup_available_regions(&main_window);
    setup_search_handler(&main_window);
    setup_download_handler(&main_window);
    setup_local_file_handler(&main_window);
    setup_device_selection_handlers(&main_window);

    main_window.run()
}

fn get_all_states() -> Vec<&'static str> {
    vec![
        "Alabama", "Alaska", "Arizona", "Arkansas", "California", "Colorado",
        "Connecticut", "Delaware", "Florida", "Georgia", "Hawaii", "Idaho",
        "Illinois", "Indiana", "Iowa", "Kansas", "Kentucky", "Louisiana",
        "Maine", "Maryland", "Massachusetts", "Michigan", "Minnesota",
        "Mississippi", "Missouri", "Montana", "Nebraska", "Nevada",
        "New Hampshire", "New Jersey", "New Mexico", "New York",
        "North Carolina", "North Dakota", "Ohio", "Oklahoma", "Oregon",
        "Pennsylvania", "Rhode Island", "South Carolina", "South Dakota",
        "Tennessee", "Texas", "Utah", "Vermont", "Virginia", "Washington",
        "West Virginia", "Wisconsin", "Wyoming",
    ]
}

fn setup_available_regions(main_window: &MainWindow) {
    let states = get_all_states();
    let regions: Vec<SharedString> = states.into_iter().map(SharedString::from).collect();
    let regions_model = Rc::new(slint::VecModel::from(regions));
    main_window.set_available_regions(regions_model.into());
}

fn setup_search_handler(main_window: &MainWindow) {
    let main_window_weak = main_window.as_weak();
    main_window.on_search_changed(move |search_text| {
        let search_text = search_text.to_lowercase();
        let states = get_all_states();
        let filtered: Vec<SharedString> = states
            .into_iter()
            .filter(|s| s.to_lowercase().starts_with(&search_text))
            .map(SharedString::from)
            .collect();
        
        if let Some(main_window) = main_window_weak.upgrade() {
            let filtered_model = Rc::new(slint::VecModel::from(filtered));
            main_window.set_available_regions(filtered_model.into());
        }
    });
}

fn setup_download_handler(main_window: &MainWindow) {
    let main_window_weak = main_window.as_weak();
    main_window.on_download_requested(move |region| {
        println!("Starting download for region: {}", region);
        
        let window_handle = main_window_weak.unwrap();
        window_handle.set_active_page(1);
        
        let thread_window_handle = main_window_weak.clone();
        let region_str = region.to_string();
        
        std::thread::spawn(move || {
            perform_download(&region_str, thread_window_handle);
        });
    });
}

fn get_output_path_for(input_path: &Path) -> PathBuf {
    let file_stem = input_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("map");
    let base_name = file_stem.strip_suffix(".osm").unwrap_or(file_stem);

    let common_dir = if std::path::Path::new("../common").exists() {
        "../common"
    } else {
        "common"
    };

    let state_bins_dir = std::format!("{}/state_bins", common_dir);
    if let Err(e) = std::fs::create_dir_all(&state_bins_dir) {
        eprintln!("Failed to create state_bins directory: {}", e);
    }

    std::path::PathBuf::from(std::format!("{}/{}.bin", state_bins_dir, base_name))
}

fn process_downloaded_map(path: &Path, ui_weak: slint::Weak<MainWindow>) {
    slint::invoke_from_event_loop({
        let ui_w = ui_weak.clone();
        move || {
            if let Some(ui) = ui_w.upgrade() {
                ui.set_active_page(2);
                ui.set_processing_progress(0.0);
            }
        }
    }).unwrap();
    
    let output_path = get_output_path_for(path);
    
    let ui_weak_for_cb = ui_weak.clone();
    if let Err(e) = map_preprocessor::build_map(path, &output_path, move |progress| {
        let ui_clone = ui_weak_for_cb.clone();
        slint::invoke_from_event_loop(move || {
            if let Some(ui) = ui_clone.upgrade() {
                ui.set_processing_progress(progress);
            }
        }).unwrap();
    }) {
        eprintln!("Failed to process map: {}", e);
    } else {
        slint::invoke_from_event_loop(move || {
            if let Some(ui) = ui_weak.upgrade() {
                refresh_devices(&ui);
                ui.set_active_page(3); // Go to Device Selection
            }
        }).unwrap();
    }
}

fn refresh_devices(main_window: &MainWindow) {
    let mut removable_devices = Vec::new();
    let mut non_removable_devices = Vec::new();
    
    if let Ok(disks) = livedisk::enumerate() {
        for disk in disks {
            // Convert bytes to GB
            let size_gb = disk.size_bytes as f64 / 1_073_741_824.0;
            let size_str = std::format!("{:.1} GB", size_gb);
            
            let device = StorageDevice {
                name: disk.name.into(),
                size: size_str.into(),
                is_removable: disk.removable,
                device_path: disk.device_path.into(),
            };
            
            if disk.removable {
                removable_devices.push(device);
            } else {
                non_removable_devices.push(device);
            }
        }
    }
    
    let rem_model = Rc::new(slint::VecModel::from(removable_devices));
    let non_rem_model = Rc::new(slint::VecModel::from(non_removable_devices));
    
    main_window.set_removable_devices(rem_model.into());
    main_window.set_non_removable_devices(non_rem_model.into());
}

fn setup_device_selection_handlers(main_window: &MainWindow) {
    let main_window_weak = main_window.as_weak();
    
    main_window.on_device_refresh_requested(move || {
        if let Some(ui) = main_window_weak.upgrade() {
            refresh_devices(&ui);
        }
    });
    
    let flash_window_weak = main_window.as_weak();
    main_window.on_flash_requested(move |device_name| {
        if let Some(ui) = flash_window_weak.upgrade() {
            ui.set_active_page(6); // Go to Flashing
            
            let mut exe_path = std::env::current_exe().unwrap_or_default();
            exe_path.pop();
            exe_path.push("os");
            
            // Try to find the image
            let mut found_img = None;
            if let Ok(entries) = std::fs::read_dir("result/sd-image") {
                for entry in entries.flatten() {
                    let path = entry.path();
                    if let Some(ext) = path.extension().and_then(|s| s.to_str()) {
                        if ext == "img" || ext == "zst" {
                            found_img = Some(path);
                            break;
                        }
                    }
                }
            }
            
            let img_path = match found_img {
                Some(p) => p,
                None => {
                    eprintln!("Could not find image in result/sd-image");
                    ui.set_active_page(4);
                    return;
                }
            };
            
            let dev_name = device_name.to_string();
            let ui_clone = flash_window_weak.clone();
            
            std::thread::spawn(move || {
                #[cfg(target_os = "linux")]
                let mut child = {
                    use std::io::IsTerminal;
                    // If launched from a terminal, developers prefer `sudo` (caches password, uses expected PAM).
                    // If launched from a GUI/shortcut, `sudo` would hang invisibly, so we must use `pkexec`.
                    let is_tty = std::io::stdin().is_terminal();
                    let (cmd_name, fallback) = if is_tty {
                        ("sudo", "pkexec")
                    } else {
                        ("pkexec", "sudo")
                    };

                    let mut cmd = std::process::Command::new(cmd_name);
                    cmd.arg(exe_path.clone()).arg(img_path.clone()).arg(&dev_name);
                    cmd.stdout(std::process::Stdio::piped());
                    cmd.stderr(std::process::Stdio::inherit());
                    
                    match cmd.spawn() {
                        Ok(c) => c,
                        Err(_) => {
                            std::process::Command::new(fallback)
                                .arg(exe_path).arg(img_path).arg(&dev_name)
                                .stdout(std::process::Stdio::piped())
                                .stderr(std::process::Stdio::inherit())
                                .spawn()
                                .expect("Failed to spawn flasher")
                        }
                    }
                };

                #[cfg(target_os = "macos")]
                let mut child = {
                    let script = format!("do shell script \"'{}' '{}' '{}'\" with administrator privileges", 
                        exe_path.display(), img_path.display(), dev_name);
                    std::process::Command::new("osascript")
                        .arg("-e").arg(script)
                        .stdout(std::process::Stdio::piped())
                        .stderr(std::process::Stdio::inherit())
                        .spawn()
                        .expect("Failed to spawn flasher")
                };

                #[cfg(target_os = "windows")]
                let mut child = {
                    // Note: capturing stdout from an elevated PowerShell process requires named pipes.
                    // For now, we spawn directly and assume the installer was run as Administrator.
                    std::process::Command::new(exe_path)
                        .arg(img_path).arg(&dev_name)
                        .stdout(std::process::Stdio::piped())
                        .stderr(std::process::Stdio::inherit())
                        .spawn()
                        .expect("Failed to spawn flasher (Please run Installer as Administrator on Windows)")
                };

                if let Some(stdout) = child.stdout.take() {
                    use std::io::{BufRead, BufReader};
                    let reader = BufReader::new(stdout);
                    for line in reader.lines().flatten() {
                        if let Some(stripped) = line.strip_prefix("PROGRESS: ") {
                            if let Ok(progress) = stripped.parse::<f32>() {
                                let ui_weak = ui_clone.clone();
                                let _ = slint::invoke_from_event_loop(move || {
                                    if let Some(ui) = ui_weak.upgrade() {
                                        ui.set_flash_progress(progress / 100.0);
                                    }
                                });
                            }
                        } else {
                            println!("OS Worker: {}", line);
                        }
                    }
                }
                
                let status = child.wait().unwrap();
                if status.success() {
                    let _ = slint::invoke_from_event_loop(move || {
                        if let Some(ui) = ui_clone.upgrade() {
                            ui.set_active_page(4); // Complete
                        }
                    });
                } else {
                    eprintln!("Flashing failed!");
                }
            });
        }
    });
}

fn setup_local_file_handler(main_window: &MainWindow) {
    let main_window_weak = main_window.as_weak();
    main_window.on_local_file_requested(move || {
        let ui_weak = main_window_weak.clone();
        std::thread::spawn(move || {
            if let Some(path) = rfd::FileDialog::new()
                .add_filter("OSM PBF Map", &["osm.pbf", "pbf"])
                .pick_file()
            {
                println!("Selected local file: {:?}", path);
                process_downloaded_map(&path, ui_weak);
            }
        });
    });
}

fn perform_download(region_str: &str, ui_handle: slint::Weak<MainWindow>) {
    let file_name = region_str.to_lowercase().replace(" ", "-");
    let url = std::format!("https://download.geofabrik.de/north-america/us/{}-latest.osm.pbf", file_name);
    println!("Downloading from: {}", url);
    
    let client = reqwest::blocking::Client::builder()
        .user_agent("TakumiSpeedAlertInstaller/1.0")
        .build()
        .expect("Failed to build HTTP client");
    
    match client.get(&url).send() {
        Ok(response) => {
            if response.status().is_success() {
                stream_download_to_file(response, ui_handle);
            } else {
                eprintln!("Failed to download. HTTP Status: {}", response.status());
            }
        }
        Err(e) => eprintln!("Request failed: {}", e),
    }
}

fn stream_download_to_file(mut response: reqwest::blocking::Response, ui_handle: slint::Weak<MainWindow>) {
    let total_size = response.content_length().unwrap_or(0);
    
    let final_filename = response.url().path_segments()
        .and_then(|segments| segments.last())
        .unwrap_or("downloaded.osm.pbf")
        .to_string();

    let mut file = match std::fs::File::create(&final_filename) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("Failed to create file {}: {}", final_filename, e);
            return;
        }
    };

    use std::io::{Read, Write};
    let mut buffer = [0; 65536]; // 64KB chunk
    let mut downloaded: u64 = 0;
    let start_time = std::time::Instant::now();
    let mut last_ui_update = std::time::Instant::now();
    
    loop {
        match response.read(&mut buffer) {
            Ok(0) => break, // EOF
            Ok(n) => {
                if let Err(e) = file.write_all(&buffer[0..n]) {
                    eprintln!("Failed to write to file {}: {}", final_filename, e);
                    break;
                }
                downloaded += n as u64;
                
                // Update UI every 100ms
                if last_ui_update.elapsed().as_millis() > 100 {
                    last_ui_update = std::time::Instant::now();
                    update_download_progress_ui(&ui_handle, downloaded, total_size, start_time);
                }
            }
            Err(e) => {
                eprintln!("Error reading from response: {}", e);
                break;
            }
        }
    }
    
    println!("Successfully downloaded to {}", final_filename);
    
    let pbf_path = std::path::PathBuf::from(final_filename);
    process_downloaded_map(&pbf_path, ui_handle);
}

fn update_download_progress_ui(
    ui_handle: &slint::Weak<MainWindow>,
    downloaded: u64,
    total_size: u64,
    start_time: std::time::Instant,
) {
    let elapsed = start_time.elapsed().as_secs_f32();
    let speed = if elapsed > 0.0 {
        (downloaded as f32 / 1024.0 / 1024.0) / elapsed
    } else {
        0.0
    };
    
    let progress = if total_size > 0 {
        downloaded as f32 / total_size as f32
    } else {
        0.0
    };
    
    let status_str = if total_size > 0 {
        std::format!("{:.1} MB / {:.1} MB", downloaded as f32 / 1024.0 / 1024.0, total_size as f32 / 1024.0 / 1024.0)
    } else {
        std::format!("{:.1} MB downloaded", downloaded as f32 / 1024.0 / 1024.0)
    };
    
    let speed_str = std::format!("{:.1} MB/s", speed);
    
    let ui_weak = ui_handle.clone();
    slint::invoke_from_event_loop(move || {
        if let Some(ui) = ui_weak.upgrade() {
            ui.set_download_progress(progress);
            ui.set_download_status(status_str.into());
            ui.set_download_speed(speed_str.into());
        }
    }).unwrap();
}
