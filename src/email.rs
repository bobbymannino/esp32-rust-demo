use esp_idf_svc::http::{client::EspHttpConnection, Headers};
use esp_idf_sys::esp_crt_bundle_attach;
use log::info;

const UNSEND_KEY: &str = "us_idcmsu4xm8_644335c2368608799f63242090250d65";

pub fn send_email() {
    info!("Sending email...");

    let mut config = esp_idf_svc::http::client::Configuration::default();
    // Attach the ESP-IDF built-in certificate bundle so TLS server certs are validated.
    // Requires sdkconfig enabling MBEDTLS_CERTIFICATE_BUNDLE.
    config.crt_bundle_attach = Some(esp_crt_bundle_attach);
    let mut client = EspHttpConnection::new(&config).expect("Failed to create client");
    info!("Client created");

    let uri = "https://send.bobman.dev/api/v1/emails";
    let mut headers: Headers = Headers::new();
    headers.set("authorization", UNSEND_KEY);
    headers.set("content-type", "application/json");

    // Minimal JSON payload (adjust as needed or parameterize the function)
    let body = "{\"to\":\"manninobobby@icloud.com\",\"from\":\"bob@bobman.dev.com\",\"subject\":\"ESP32 Test\",\"text\":\"Hello from ESP32 over Rust!\"}";
    let content_length = body.len().to_string();
    headers.set("content-length", &content_length);
    info!("Headers set");

    client
        .initiate_request(esp_idf_svc::http::Method::Post, uri, &headers.as_slice())
        .expect("Failed to initiate POST request");
    info!("Request initiated");

    // Write request body
    client
        .write_all(body.as_bytes())
        .expect("Failed to write request body");
    info!("Request body written");

    // Optionally read a portion of the response (may block until server responds)
    let mut buf = [0u8; 256];
    if let Ok(len) = client.read(&mut buf) {
        if len > 0 {
            if let Ok(snippet) = core::str::from_utf8(&buf[..len]) {
                info!("response (truncated): {}", snippet);
            } else {
                info!("response (non-UTF8, {} bytes)", len);
            }
        }
    }

    info!("status: {}", client.status().to_string());

    // curl --request POST \
    //   --url https://send.bobman.dev/api/v1/emails \
    //   --header 'Content-Type: application/json' \
    //   --data '{
    //   "to": "<string>",
    //   "from": "<string>",
    //   "subject": "<string>",
    //   "text": "<string>",
    //   }'
    // }'
}
