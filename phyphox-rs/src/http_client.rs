use reqwest::Client as ReqwestClient;
use serde_json::Value;
use std::time::Duration;

use crate::errors::PhyphoxError;

const CLIENT_TIMEOUT_DEFAULT: u64 = 5;

pub(crate) struct HttpClient {
    client: ReqwestClient,
    base_url: String,
}

impl HttpClient {
    pub(crate) fn new(base_url: String) -> Result<Self, PhyphoxError> {
        let client = ReqwestClient::builder()
            .timeout(Duration::from_secs(CLIENT_TIMEOUT_DEFAULT))
            .build()
            .map_err(|e| PhyphoxError::ClientBuild(e.to_string()))?;

        Ok(Self { client, base_url })
    }

    pub(crate) async fn fetch_json(&self, path: &str) -> Result<Value, PhyphoxError> {
        // Example of data returned by Phyphox
        //  /get?acc_time=2.0412&accX=2.0412|acc_time&accY=2.0412|acc_time&accZ=2.0412|acc_time,
        // {"buffer":{"accX":{"buffer":[0.17797988891601563,0.16510665893554688,0.18232086181640625,0.1778302001953125],"size":0,"updateMode":"partial"},
        //            "accY":{"buffer":[0.2908451843261719,0.26614654541015625,0.27228378295898437,0.28380981445312503],"size":0,"updateMode":"partial"},
        //            "accZ":{"buffer":[9.756411437988282,9.761650543212891,9.768985290527345,9.750723266601563],"size":0,"updateMode":"partial"},
        //            "acc_time":{"buffer":[2.5006562499329448,3.001219541300088,3.501783541403711,4.00234662508592],"size":0,"updateMode":"partial"}},
        // "status":{"countDown":0,"measuring":true,"session":"11114880","timedRun":false}}
        let url = format!("{}{}", self.base_url, path);
        let response = self
            .client
            .get(&url)
            .send()
            .await
            .map_err(|e| PhyphoxError::FetchData(e.to_string()))?;

        let json: Value = response
            .json()
            .await
            .map_err(|e| PhyphoxError::FetchData(e.to_string()))?;
        Ok(json)
    }
}
