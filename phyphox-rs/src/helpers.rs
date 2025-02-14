use serde_json::Value;
use uuid::Uuid;

use common::types::sensors::sensor_type;
use common::types::sensors::SensorType;
use common::types::untimed::xyz::N_XYZ_COORDINATES;
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
) -> Result<(&'static str, [&'static str; N_XYZ_COORDINATES], usize), PhyphoxError> {
    match sensor {
        sensor_type::ACCELEROMETER_OFFSET..sensor_type::GYROSCOPE_OFFSET => {
            Ok((ACC_TIME, ACC_VARIABLES, 0))
        }
        sensor_type::GYROSCOPE_OFFSET..sensor_type::MAGNETOMETER_OFFSET => {
            Ok((GYRO_TIME, GYRO_VARIABLES, 1))
        }
        sensor_type::MAGNETOMETER_OFFSET..sensor_type::OTHER_OFFSET => {
            Ok((MAG_TIME, MAG_VARIABLES, 2))
        }
        _ => Err(PhyphoxError::Other(format!(
            "Sensor {} doesnt exist",
            sensor
        ))),
    }
}

pub(crate) fn update_measurement_time(data: &[f64], timestamp: &mut f64) {
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
pub(crate) fn combine_results(
    results: Vec<Vec<f64>>,
    timestamp_at_boot: f64,
) -> (Vec<f64>, Vec<XYZ>) {
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

        if let Ok(xyz) = XYZ::try_from(values) {
            untimed_data.push(xyz);
            timestamp.push(results[0][row] + timestamp_at_boot);
        }
    }
    (timestamp, untimed_data)
}

pub(crate) fn build_query(variables: &[&str], time_var: &str, since: Option<f64>) -> String {
    let mut query = time_var.to_string();
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

pub(crate) fn to_type_id(sensor_type: &str, sensor_id_cluster: &[Uuid]) -> Option<String> {
    let sensor_type = sensor_type.to_lowercase();

    if sensor_type.contains("acc") {
        Some(format!("Accelerometer::{}", sensor_id_cluster[0]))
    } else if sensor_type.contains("gyr") {
        Some(format!("Gyroscope::{}", sensor_id_cluster[1]))
    } else if sensor_type.contains("mag") {
        Some(format!("Magnetometer::{}", sensor_id_cluster[2]))
    } else {
        None
    }
}

pub(crate) fn extract_uuids(sensors: &[SensorType]) -> Vec<Uuid> {
    let mut uuids = vec![Uuid::nil(), Uuid::nil(), Uuid::nil()]; // Default with nil UUIDs

    for sensor in sensors {
        match sensor {
            SensorType::Accelerometer(uuid) => uuids[0] = *uuid,
            SensorType::Gyroscope(uuid) => uuids[1] = *uuid,
            SensorType::Magnetometer(uuid) => uuids[2] = *uuid,
            _ => {} // Ignore other sensor types (if needed)
        }
    }

    uuids
}
#[cfg(test)]
mod tests {
    use super::*;
    use rand::Rng;
    use serde_json::json;

    fn generate_random_integer(x: usize, y: usize) -> usize {
        let mut rng = rand::thread_rng();
        rng.gen_range(x..y) as usize
    }
    #[test]
    fn test_control_str() {
        for _ in 0..100 {
            let idx = generate_random_integer(
                sensor_type::ACCELEROMETER_OFFSET,
                sensor_type::GYROSCOPE_OFFSET,
            );
            assert_eq!(control_str(idx).unwrap(), (ACC_TIME, ACC_VARIABLES, 0));

            let idx = generate_random_integer(
                sensor_type::GYROSCOPE_OFFSET,
                sensor_type::MAGNETOMETER_OFFSET,
            );
            assert_eq!(control_str(idx).unwrap(), (GYRO_TIME, GYRO_VARIABLES, 1));

            let idx = generate_random_integer(
                sensor_type::MAGNETOMETER_OFFSET,
                sensor_type::OTHER_OFFSET,
            );
            assert_eq!(control_str(idx).unwrap(), (MAG_TIME, MAG_VARIABLES, 2));

            let idx = generate_random_integer(sensor_type::OTHER_OFFSET, sensor_type::MAX_OFFSET);
            assert!(control_str(idx).is_err());
        }
    }

    #[test]
    fn test_update_measurement_time() {
        let mut timestamp = 0.0;
        update_measurement_time(&[1.0, 2.0, 3.0], &mut timestamp);
        assert_eq!(timestamp, 3.0 + EPS_MEASUREMENT_TIME);
    }

    #[test]
    fn test_get_status_from_json() {
        let data = json!({
            "status": {
                "measuring": true
            }
        });

        assert!(get_status_from_json(&data).unwrap());

        let data = json!({
            "status": {
                "measuring": false
            }
        });
        assert!(!get_status_from_json(&data).unwrap());

        let data = json!({});
        assert!(get_status_from_json(&data).is_err());
    }

    #[test]
    fn test_parse_results() {
        let data = json!({
            "buffer": {
                "acc_time": {
                    "buffer": [1.0, 2.0, 3.0]
                },
                "accX": {
                    "buffer": [0.1, 0.2, 0.3]
                },
                "accY": {
                    "buffer": [0.4, 0.5, 0.6]
                },
                "accZ": {
                    "buffer": [0.7, 0.8, 0.9]
                }
            }
        });
        let variables = ["accX", "accY", "accZ"];
        let results = parse_results(&data, &variables, "acc_time").unwrap();
        assert_eq!(results.len(), 4);
        assert_eq!(results[0], vec![1.0, 2.0, 3.0]);
        assert_eq!(results[1], vec![0.1, 0.2, 0.3]);
        assert_eq!(results[2], vec![0.4, 0.5, 0.6]);
        assert_eq!(results[3], vec![0.7, 0.8, 0.9]);
    }

    #[test]
    fn test_combine_results() {
        let results = vec![
            vec![1.0, 2.0, 3.0],
            vec![0.1, 0.2, 0.3],
            vec![0.4, 0.5, 0.6],
            vec![0.7, 0.8, 0.9],
        ];
        let (timestamps, untimed_data) = combine_results(results, 0.0);
        assert_eq!(timestamps, vec![1.0, 2.0, 3.0]);
        assert_eq!(untimed_data.len(), 3);
    }

    #[test]
    fn test_build_query() {
        let variables = ["accX", "accY", "accZ"];
        let query = build_query(&variables, "acc_time", Some(1.0));
        assert_eq!(
            query,
            "acc_time=1.0000&accX=1.0000|acc_time&accY=1.0000|acc_time&accZ=1.0000|acc_time"
        );

        let query = build_query(&variables, "acc_time", None);
        assert_eq!(query, "acc_time&accX&accY&accZ");
    }
}
