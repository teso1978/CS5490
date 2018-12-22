{
    "targets": [
        {
            "target_name": "CS5490",
            "include_dirs": [ "<!(node -e \"require('nan')\")" ],
            "sources": [ "CS5490.cc"],
            "link_settings": { "libraries": [ "-lwiringPi" ] }
        }
    ]
}