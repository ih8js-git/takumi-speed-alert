use slint::*;
use std::rc::Rc;

slint::include_modules!();

fn main() -> Result<(), slint::PlatformError> {
    let main_window = MainWindow::new()?;

    setup_available_regions(&main_window);
    setup_search_handler(&main_window);
    setup_download_handler(&main_window);

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
    
    slint::invoke_from_event_loop(move || {
        if let Some(ui) = ui_handle.upgrade() {
            ui.set_active_page(2);
        }
    }).unwrap();
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
