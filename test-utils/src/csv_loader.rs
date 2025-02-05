use std::error::Error;

use csv::Reader;

pub fn load_csv(file_path: &str) -> Result<Vec<Vec<f64>>, Box<dyn Error>> {
    let mut rdr = Reader::from_path(file_path)?;
    let mut data = Vec::new();

    for result in rdr.records() {
        let record = result?;
        let row: Vec<f64> = record
            .iter()
            .filter_map(|s| s.parse::<f64>().ok())
            .collect();
        data.push(row);
    }

    Ok(data)
}

pub fn load_csv_columns<T: TryFrom<Vec<f64>>>(
    file_path: &str,
    columns: &[usize],
) -> Result<Vec<T>, Box<dyn Error>> {
    if columns.is_empty() {
        return Err("No columns provided".into());
    }

    let data = load_csv(file_path)?;

    let result = data
        .into_iter()
        .map(|rows| {
            columns
                .iter()
                .map(|&i| {
                    rows.get(i)
                        .ok_or_else(|| format!("Column index {} out of bounds", i).into())
                        .map(|&val| val)
                })
                .collect::<Result<Vec<f64>, Box<dyn Error>>>()
        })
        .collect::<Result<Vec<Vec<f64>>, Box<dyn Error>>>()?
        .into_iter()
        .map(|f64_values| {
            T::try_from(f64_values).map_err(|_| format!("Failed to convert to T").into())
        })
        .collect::<Result<Vec<T>, Box<dyn Error>>>()?;

    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Sample3D;

    #[test]
    fn test_read_csv() {
        let file_name = "./test_data/sensor_readings.csv";
        load_csv(file_name).unwrap();
    }

    #[test]
    #[should_panic(expected = "No such file or directory")]
    fn test_read_inexistent_csv() {
        let file_name = "./test_data/sensor_readingss.csv"; // Path to the non-existent CSV file
        let _ = load_csv(file_name).unwrap(); // This should panic because the file doesn't exist
    }

    #[test]
    fn test_load_csv_correct_columns() {
        let file_name = "./test_data/sensor_readings.csv";
        let columns = vec![0, 1, 2];
        let data = load_csv_columns::<Vec<f64>>(file_name, &columns).unwrap();

        assert!(data[0].len() == columns.len());
    }

    #[test]
    #[should_panic]
    fn test_load_csv_incorrect_columns() {
        let file_name = "./test_data/sensor_readings.csv";
        let columns = vec![0, 1, 20];
        load_csv_columns::<Vec<f64>>(file_name, &columns).unwrap();
    }

    #[test]
    fn test_load_csv_as_sample3d() {
        let file_name = "./test_data/sensor_readings.csv";
        let columns = vec![0, 1, 2, 3];
        load_csv_columns::<Sample3D>(file_name, &columns).unwrap();
    }

    #[test]
    #[should_panic(expected = "Failed to convert to T")]
    fn test_fail_load_csv_as_sample3d() {
        let file_name = "./test_data/sensor_readings.csv";
        let columns = vec![0, 1, 2];
        load_csv_columns::<Sample3D>(file_name, &columns).unwrap();
    }
}
