use esp_idf_svc::http::{client::EspHttpConnection, Headers};
use log::info;

const UNSEND_KEY: &str = "us_idcmsu4xm8_644335c2368608799f63242090250d65";

pub fn send_email() {
    let config = esp_idf_svc::http::client::Configuration::default();
    let mut client = EspHttpConnection::new(&config).expect("Failed to create client");

    let uri = "https://send.bobman.dev/api/v1/emails";
    let mut headers: Headers = Headers::new();
    headers.set("authorization", UNSEND_KEY);

    client
        .initiate_request(esp_idf_svc::http::Method::Post, uri, &headers.as_slice())
        .expect("Failed to do post request");

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
