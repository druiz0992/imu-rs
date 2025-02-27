use gnuplot::PlotOption::LineWidth;
use gnuplot::{AxesCommon, Caption, Color, Figure};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use uuid::Uuid;

use common::traits::{IMUReadings, IMUSample, IMUSink, IMUSource};
use common::types::buffers::CircularBuffer;
use common::types::clock::Clock;
use common::types::sensors::{SensorReadings, SensorType};
use common::types::timed::Sample3D;
use publisher::{listener, Listener};

type PlotDataVec = (
    CircularBuffer<f64>,
    CircularBuffer<f64>,
    CircularBuffer<f64>,
    CircularBuffer<f64>,
);
type PlotData = (f64, f64, f64, f64);

pub struct PlotManager {
    figure: HashMap<SensorType, Figure>,
    plots: HashMap<SensorType, PlotDataVec>,
    start_time: f64,
    sensor_cluster: Vec<SensorType>,
    window_size: usize,
    tag: String,
}

impl PlotManager {
    pub fn new(tag: &str, sensor_cluster: Vec<SensorType>, window_size: usize) -> Arc<Mutex<Self>> {
        Arc::new(Mutex::new(Self {
            figure: HashMap::new(),
            sensor_cluster,
            plots: HashMap::new(),
            start_time: Clock::now().as_secs(),
            window_size,
            tag: tag.to_string(),
        }))
    }

    fn add_plot(&mut self, sensor_type: &SensorType) {
        if self.plots.contains_key(sensor_type) {
            return;
        }

        self.figure.insert(sensor_type.clone(), Figure::new());

        self.plots.insert(
            sensor_type.clone(),
            (
                CircularBuffer::new(self.window_size),
                CircularBuffer::new(self.window_size),
                CircularBuffer::new(self.window_size),
                CircularBuffer::new(self.window_size),
            ),
        );
    }

    fn add_sample(&mut self, sensor_type: &SensorType, data: PlotData) {
        if let Some((t_vals, x_vals, y_vals, z_vals)) = self.plots.get_mut(sensor_type) {
            t_vals.push(data.0);
            x_vals.push(data.1);
            y_vals.push(data.2);
            z_vals.push(data.3);
        }
    }

    pub fn update(&mut self) {
        for sensor_type in &self.sensor_cluster {
            if let Some((t_vals, x_vals, y_vals, z_vals)) = self.plots.get(sensor_type) {
                if let Some(figure) = self.figure.get_mut(sensor_type) {
                    figure.clear_axes();
                    let axes = figure.axes2d();
                    axes.set_title(&format!("{} - {:?}", self.tag, sensor_type), &[]);
                    axes.set_x_label("Time (ms)", &[]);
                    axes.set_y_label("Measurements", &[]);

                    axes.lines(
                        t_vals,
                        x_vals,
                        &[Color("blue"), LineWidth(2.0), Caption("X")],
                    );
                    axes.lines(
                        t_vals,
                        y_vals,
                        &[Color("green"), LineWidth(2.0), Caption("Y")],
                    );
                    axes.lines(
                        t_vals,
                        z_vals,
                        &[Color("red"), LineWidth(2.0), Caption("Z")],
                    );

                    figure.show_and_keep_running().unwrap();
                }
            }
        }
    }
}

#[derive(Clone)]
pub struct Plot1D(Arc<Mutex<PlotManager>>);

impl Plot1D {
    pub fn new(tag: &str, sensor_cluster: Vec<SensorType>, window_size: usize) -> Self {
        let pm = PlotManager::new(tag, sensor_cluster.clone(), window_size);
        let mut pm_locked = pm.lock().unwrap();

        for sensor_type in &sensor_cluster {
            pm_locked.add_plot(sensor_type);
        }

        drop(pm_locked);

        Self(pm)
    }

    pub fn start(&self, refresh_period_millis: f64) {
        let plot_manager = Arc::clone(&self.0);
        std::thread::spawn(move || {
            let period = Duration::from_secs_f64(refresh_period_millis / 1000.0);
            loop {
                let start_time = Instant::now();
                let mut plot = plot_manager.lock().unwrap();
                plot.update();
                drop(plot);

                let elapsed_time = start_time.elapsed();
                let sleep_time = if elapsed_time < period {
                    period - elapsed_time
                } else {
                    Duration::new(0, 0)
                };

                thread::sleep(sleep_time);
            }
        });
    }
}

fn secs_to_t_val(timestamp_secs: f64, base_time: f64) -> f64 {
    f64::max(0.0, timestamp_secs - base_time)
}

impl IMUSink<SensorReadings<Sample3D>, Sample3D> for Plot1D {
    fn attach_listeners(
        &self,
        source: &dyn IMUSource<SensorReadings<Sample3D>, Sample3D>,
        sensor_cluster: &[SensorType],
    ) -> Result<Vec<Uuid>, String> {
        let mut listener = listener!(self.process_samples);
        let mut ids = Vec::with_capacity(sensor_cluster.len());
        for sensor_type in sensor_cluster {
            match source.register_listener(&mut listener, sensor_type) {
                Ok(id) => {
                    ids.push(id);
                }
                Err(e) => return Err(e),
            }
        }
        Ok(ids)
    }
    fn detach_listener(
        &self,
        _source: &dyn IMUSource<SensorReadings<Sample3D>, Sample3D>,
        _id: Uuid,
    ) {
        todo!();
    }

    fn process_samples(&self, _listener_id: Uuid, samples: Arc<SensorReadings<Sample3D>>) {
        let sensor_type = samples.get_sensor_type();
        if let Some(samples) = samples.get_samples().last() {
            let mut plot = self.0.lock().unwrap();
            let timestamp = samples.get_timestamp_secs();
            let [x_val, y_val, z_val] = samples.get_measurement().inner();
            let t_val = secs_to_t_val(timestamp, plot.start_time);

            plot.add_sample(&sensor_type, (t_val, x_val, y_val, z_val));
        }
    }
}
