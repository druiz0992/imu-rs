use serde_json::Value;

use common::constants::N_XYZ_COORDINATES;
use common::types::XYZ;

use crate::models::errors::PhyphoxError;

const ACC_VARIABLES: [&str; N_XYZ_COORDINATES] = ["accX", "accY", "accZ"];
const ACC_TIME: &str = "acc_time";
const GYRO_VARIABLES: [&str; N_XYZ_COORDINATES] = ["gyroX", "gyroY", "gyroZ"];
const GYRO_TIME: &str = "gyro_time";
const MAG_VARIABLES: [&str; N_XYZ_COORDINATES] = ["magX", "magY", "magZ"];
const MAG_TIME: &str = "mag_time";

const STATUS: &str = "status";
const MEASURING: &str = "measuring";
const BUFFER: &str = "buffer";

const EPS_MEASUREMENT_TIME: f64 = 10e-5;

pub(crate) fn control_str(
    sensor: usize,
) -> Result<(&'static str, [&'static str; N_XYZ_COORDINATES]), PhyphoxError> {
    match sensor {
        0 => Ok((ACC_TIME, ACC_VARIABLES)),
        1 => Ok((GYRO_TIME, GYRO_VARIABLES)),
        2 => Ok((MAG_TIME, MAG_VARIABLES)),
        _ => Err(PhyphoxError::Other(format!(
            "Sensor {} doesnt exist",
            sensor
        ))),
    }
}

pub(crate) fn update_measurement_time(data: &Vec<f64>, timestamp: &mut f64) {
    if let Some(last_row) = data.last() {
        *timestamp = last_row + EPS_MEASUREMENT_TIME;
    }
}

pub(crate) fn get_status_from_json(data: &Value) -> Result<bool, PhyphoxError> {
    let status = data[STATUS][MEASURING]
        .as_bool()
        .ok_or(PhyphoxError::IncorrectDataFormat(
            "Missing status.measuring".to_string(),
        ))?;
    Ok(status)
}

pub(crate) fn parse_results(
    data: &Value,
    variables: &[&str],
    time_var: &str,
) -> Result<Vec<Vec<f64>>, PhyphoxError> {
    let buffers = data.get(BUFFER).ok_or(PhyphoxError::IncorrectDataFormat(
        "Missing buffer".to_string(),
    ))?;
    let mut results: Vec<Vec<f64>> = Vec::with_capacity(variables.len() + 1);
    for var in std::iter::once(time_var).chain(variables.iter().copied()) {
        let buffer = buffers.get(var).and_then(|v| v.get(BUFFER)).ok_or(
            PhyphoxError::IncorrectDataFormat(format!("Missing buffer for {}", var)),
        )?;
        let values: Vec<f64> = buffer
            .as_array()
            .ok_or(PhyphoxError::IncorrectDataFormat(format!(
                "Invalid buffer format for {}",
                var
            )))?
            .iter()
            .filter_map(|v| v.as_f64())
            .collect();
        results.push(values);
    }
    Ok(results)
}

/// Function basically transposes incoming data. Input data is structured as
/// result[0] : time
/// result[1] : x component
/// result[2] : y component
/// reults[3] : z component
///
/// and the output is
/// output[0] : [time[0], time[1]....]
/// output[1] : [x[0], y[0]]
/// output[2] : [x[1], y[1]]
/// ...
pub(crate) fn combine_results(results: Vec<Vec<f64>>) -> (Vec<f64>, Vec<XYZ>) {
    let row_count = results.len().min(4);

    let n_samples = results[0].len();
    let mut untimed_data = Vec::with_capacity(n_samples);
    let mut timestamp = Vec::with_capacity(n_samples);

    // skip time row
    for row in 0..row_count {
        let values: Vec<f64> = results
            .iter()
            .skip(1)
            .filter_map(|col| col.get(row))
            .cloned()
            .collect();

        if let Some(xyz) = XYZ::from_vec(values) {
            untimed_data.push(xyz);
            timestamp.push(results[0][row].clone());
        }
    }
    (timestamp, untimed_data)
}

pub(crate) fn build_query(variables: &[&str], time_var: &str, since: Option<f64>) -> String {
    let mut query = format!("{}", time_var);
    if let Some(since_val) = since {
        query = format!("{}={:.4}", time_var, since_val);
    }

    let variable_query: String = variables
        .iter()
        .map(|&var| {
            if since.is_some() {
                format!("&{}={:.4}|{}", var, since.unwrap(), time_var)
            } else {
                format!("&{}", var)
            }
        })
        .collect::<Vec<_>>()
        .join("");
    query.push_str(&variable_query);
    query
}
