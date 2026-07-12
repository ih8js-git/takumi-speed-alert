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

    main_window.on_download_requested(|region| {
        println!("Starting download for region: {}", region);
        let region_str = region.to_string();
        
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
                        // Extract the final filename from the redirected URL
                        let final_filename = response.url().path_segments()
                            .and_then(|segments| segments.last())
                            .unwrap_or("downloaded.osm.pbf")
                            .to_string();

                        if let Ok(mut file) = std::fs::File::create(&final_filename) {
                            if let Err(e) = std::io::copy(&mut response, &mut file) {
                                eprintln!("Failed to write to file {}: {}", final_filename, e);
                            } else {
                                println!("Successfully downloaded to {}", final_filename);
                            }
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
