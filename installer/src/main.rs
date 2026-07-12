use slint::*;
use std::rc::Rc;

slint::include_modules!();

fn main() -> Result<(), slint::PlatformError> {
    let main_window = MainWindow::new()?;

    let states = [
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
    ];
    let regions: Vec<SharedString> = states.iter().map(|&s| SharedString::from(s)).collect();
    let regions_model = Rc::new(slint::VecModel::from(regions));
    
    let main_window_weak = main_window.as_weak();
    main_window.set_available_regions(regions_model.into());

    main_window.on_search_changed(move |search_text| {
        let search_text = search_text.to_lowercase();
        let filtered: Vec<SharedString> = states
            .iter()
            .filter(|&s| s.to_lowercase().starts_with(&search_text))
            .map(|&s| SharedString::from(s))
            .collect();
        
        if let Some(main_window) = main_window_weak.upgrade() {
            let filtered_model = Rc::new(slint::VecModel::from(filtered.clone()));
            main_window.set_available_regions(filtered_model.into());
        }
    });

    let main_window_weak_for_download = main_window.as_weak();
    main_window.on_download_requested(move |region| {
        println!("Starting download for region: {}", region);
        let region_str = region.to_string();
        
        let window_handle = main_window_weak_for_download.unwrap();
        window_handle.set_active_page(1);
        
        let thread_window_handle = main_window_weak_for_download.clone();
        std::thread::spawn(move || {
            let file_name = region_str.to_lowercase().replace(" ", "-");
            let url = std::format!("https://download.geofabrik.de/north-america/us/{}-latest.osm.pbf", file_name);
            println!("Downloading from: {}", url);
            
            let client = reqwest::blocking::Client::builder()
                .user_agent("TakumiSpeedAlertInstaller/1.0")
                .build()
                .expect("Failed to build HTTP client");
            
            match client.get(&url).send() {
                Ok(mut response) => {
                    if response.status().is_success() {
                        let total_size = response.content_length().unwrap_or(0);
                        
                        let final_filename = response.url().path_segments()
                            .and_then(|segments| segments.last())
                            .unwrap_or("downloaded.osm.pbf")
                            .to_string();

                        if let Ok(mut file) = std::fs::File::create(&final_filename) {
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
                                            
                                            let ui_weak = thread_window_handle.clone();
                                            slint::invoke_from_event_loop(move || {
                                                if let Some(ui) = ui_weak.upgrade() {
                                                    ui.set_download_progress(progress);
                                                    ui.set_download_status(status_str.into());
                                                    ui.set_download_speed(speed_str.into());
                                                }
                                            }).unwrap();
                                        }
                                    }
                                    Err(e) => {
                                        eprintln!("Error reading from response: {}", e);
                                        break;
                                    }
                                }
                            }
                            
                            println!("Successfully downloaded to {}", final_filename);
                            
                            let ui_weak = thread_window_handle.clone();
                            slint::invoke_from_event_loop(move || {
                                if let Some(ui) = ui_weak.upgrade() {
                                    ui.set_active_page(2);
                                }
                            }).unwrap();
                        } else {
                            eprintln!("Failed to create file {}", final_filename);
                        }
                    } else {
                        eprintln!("Failed to download. HTTP Status: {}", response.status());
                    }
                }
                Err(e) => {
                    eprintln!("Request failed: {}", e);
                }
            }
        });
    });

    main_window.run()
}
