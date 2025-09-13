use embedded_svc::http::client::Client as HttpClient;
use embedded_svc::utils::io;
use esp_idf_hal::io::Write;
use esp_idf_svc::http::{
    client::{Configuration, EspHttpConnection},
    Headers,
};
use esp_idf_sys::esp_crt_bundle_attach;
use log::{error, info};

const UNSEND_KEY: &str = "us_idcmsu4xm8_644335c2368608799f63242090250d65";

pub fn send_email() {
    info!("Sending email...");

    let mut config = Configuration::default();
    config.crt_bundle_attach = Some(esp_crt_bundle_attach);
    info!("attached crt bundle");

    let conn = EspHttpConnection::new(&config).expect("failed to create connection");

    let mut client = HttpClient::wrap(conn);

    let url = "https://send.bobman.dev/api/v1/emails";

    let mut headers: Headers = Headers::new();
    let auth_value = format!("Bearer: {}", UNSEND_KEY);
    headers.set("Authorization", auth_value.as_str());
    headers.set("content-type", "application/json");

    // Minimal JSON payload (adjust as needed or parameterize the function)
    let body = b"{\"to\":\"manninobobby@icloud.com\",\"from\":\"bob@bobman.dev\",\"subject\":\"ESP32 Test\",\"text\":\"Hello from ESP32 over Rust!\"}";
    let content_length = body.len().to_string();
    headers.set("content-length", &content_length);
    info!("Headers set");

    let mut req = client
        .post(url, &headers.as_slice())
        .expect("failed to POST");

    req.write_all(body).expect("failed to write body");
    req.flush().expect("failed to flush");

    let mut res = req.submit().expect("failed to submit");

    let status = res.status();
    info!("status of POST returned is {status}");

    let mut buf = [0u8; 1024];
    let bytes_read = io::try_read_full(&mut res, &mut buf)
        .map_err(|e| e.0)
        .expect("failed to read res body");
    info!("Read {bytes_read} bytes");

    match std::str::from_utf8(&buf[0..bytes_read]) {
        Ok(body_string) => info!(
            "Response body (truncated to {} bytes): {:?}",
            buf.len(),
            body_string
        ),
        Err(e) => error!("Error decoding response body: {e}"),
    };
}
