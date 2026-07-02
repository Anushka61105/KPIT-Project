#----------------------Live detection using RPI-------------------
# RPI Live detection Code
# live detection code in this file 

 nano ~/Documents/archive/car/runs/live_detect.py 

# live detection start

source ~/yolo-env/bin/activate

python3 ~/Documents/archive/car/runs/live_detect.py

#Actual live detection code send via CAN to stm32 in nano file 
from ultralytics import YOLO
from picamera2 import Picamera2
import can
import cv2
import time

bus = can.interface.Bus(channel='can0', interface='socketcan')

model = YOLO("/home/raspberrypi5/runs/detect/train-22/weights/best.pt")

sign_codes = {
    "green_light": 1,
    "red_light": 2,
    "speed_limit_20": 3,
    "speed_limit_30": 4,
    "speed_limit_40": 5,
    "speed_limit_50": 6,
    "speed_limit_60": 7,
    "stop": 8,
}

SEND_INTERVAL = 2.0
STABLE_FRAMES = 5

last_send_time = 0
candidate_code = 0
candidate_count = 0
candidate_name = "none"

picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

print("Traffic Sign Detection + CAN Transmission Started")

try:
    while True:
        frame = picam2.capture_array()

        results = model(frame, conf=0.35, verbose=False)

        current_code = 0
        current_name = "none"

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                raw_name = model.names[cls_id]

                class_name = raw_name.lower().replace(" ", "_").replace("-", "_")

                print("Detected:", class_name)

                if class_name in sign_codes:
                    current_code = sign_codes[class_name]
                    current_name = class_name
                    break

            if current_code != 0:
                break

        if current_code == candidate_code and current_code != 0:
            candidate_count += 1
        else:
            candidate_code = current_code
            candidate_name = current_name
            candidate_count = 1

        now = time.time()

        if (
            candidate_code != 0
            and candidate_count >= STABLE_FRAMES
            and now - last_send_time >= SEND_INTERVAL
        ):
            msg = can.Message(
                arbitration_id=0x100,
                data=[candidate_code],
                is_extended_id=False
            )

            bus.send(msg)

            print(f"CAN Sent: {candidate_name} | Code: {candidate_code}")

            last_send_time = now
            candidate_count = 0

        annotated_frame = results[0].plot()
        cv2.imshow("Traffic Sign Detection", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Program Stopped")

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    bus.shutdown()
    print("CAN Bus Closed Properly")

# Auto Start Live detection cmd without PC in this file 

sudo nano /etc/systemd/system/traffic_sign.service

#Auto start Live detection cmd without PC

[Unit]
Description=Traffic Sign Detection Auto Start
After=multi-user.target
Wants=multi-user.target

[Service]
User=raspberrypi5
WorkingDirectory=/home/raspberrypi5/Documents/archive/car/runs
ExecStartPre=/bin/sleep 25
ExecStart=/home/raspberrypi5/yolo-env/bin/python3 /home/raspberrypi5/Documents/archive/car/runs/live_detect.py
Restart=always
RestartSec=15

[Install]
WantedBy=multi-user.target

#Load the code and start, status, stop cmds 

sudo systemctl daemon-reload

sudo systemctl enable traffic_sign.service

sudo systemctl restart traffic_sign.service

sudo systemctl status traffic_sign.service

sudo systemctl stop traffic_sign.service

#Check live detection log 

journalctl -u traffic_sign.service -f


#-----------------------------STM32 Code------------------------------------
#STM complete code start here (all 4 motor on+ can receive live detection from rpi + speed compliance/reduction +can send to rpi to save csv)


#include <SPI.h>
#include <mcp2515.h>

// ---------- MOTOR CONTROL PINS ----------
// Shared by BOTH L293Ds

#define ENA1 D3
#define IN1  D4
#define IN2  D5

#define HALL_PIN D6

#define ENA2 D9
#define IN3  D7
#define IN4  D8

#define MAGNETS 4
#define RPM_INTERVAL 1000

MCP2515 mcp2515(10);

struct can_frame rxMsg;
struct can_frame txMsg;

volatile unsigned long pulseCount = 0;

float rpm = 0;
int speedLimitRPM = 320;
int pwmValue = 220;

uint8_t currentSignCode = 0;
uint8_t statusCode = 0;
// 0 = NORMAL
// 1 = OVERSPEED
// 2 = STOP

unsigned long previousMillis = 0;

// --------------------------------------------------
// HALL SENSOR ISR
// --------------------------------------------------
void hallISR()
{
  pulseCount++;
}

// --------------------------------------------------
// SIGN NAME
// --------------------------------------------------
String getSignName(uint8_t code)
{
  switch (code)
  {
    case 1: return "GREEN_LIGHT";
    case 2: return "RED_LIGHT";
    case 3: return "SPEED_LIMIT_20";
    case 4: return "SPEED_LIMIT_30";
    case 5: return "SPEED_LIMIT_40";
    case 6: return "SPEED_LIMIT_50";
    case 7: return "SPEED_LIMIT_60";
    case 8: return "STOP";
    default: return "UNKNOWN";
  }
}

// --------------------------------------------------
// RPM LIMIT FOR EACH SIGN
// --------------------------------------------------
int getRPMLimit(uint8_t code)
{
  switch (code)
  {
    case 1: return 320;
    case 2: return 0;
    case 3: return 80;
    case 4: return 120;
    case 5: return 160;
    case 6: return 200;
    case 7: return 240;
    case 8: return 0;
    default: return 320;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(2000);

  // Motor Outputs
  pinMode(ENA1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENA2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Hall Sensor
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

  // Forward Direction
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Initial Speed
  analogWrite(ENA1, pwmValue);
  analogWrite(ENA2, pwmValue);

  // CAN Setup
  SPI.begin();
  mcp2515.reset();

  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK)
  {
    Serial.println("CAN Init Failed");
    while (1);
  }

  mcp2515.setNormalMode();

  Serial.println("STM32 Four Motor Speed Compliance Ready");
}

void loop()
{
  // --------------------------------------------------
  // RECEIVE SIGN FROM RPI
  // --------------------------------------------------
  if (mcp2515.readMessage(&rxMsg) == MCP2515::ERROR_OK)
  {
    if (rxMsg.can_id == 0x100)
    {
      currentSignCode = rxMsg.data[0];
      speedLimitRPM = getRPMLimit(currentSignCode);

      Serial.println();
      Serial.print("Sign Received : ");
      Serial.println(getSignName(currentSignCode));

      Serial.print("RPM Limit     : ");
      Serial.println(speedLimitRPM);
    }
  }

  // --------------------------------------------------
  // RPM CALCULATION EVERY SECOND
  // --------------------------------------------------
  if (millis() - previousMillis >= RPM_INTERVAL)
  {
    unsigned long interval = millis() - previousMillis;
    previousMillis = millis();

    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    rpm = (pulses / (float)MAGNETS) * (60000.0 / interval);

    // --------------------------------------------------
    // SPEED COMPLIANCE
    // --------------------------------------------------

    if (speedLimitRPM == 0)
    {
      pwmValue = 0;
      statusCode = 2;
    }
    else if (rpm > speedLimitRPM)
    {
      pwmValue -= 15;

      if (pwmValue < 120)
      {
        pwmValue = 120;
      }

      statusCode = 1;
    }
    else
    {
      if (pwmValue < 220)
      {
        pwmValue += 10;

        if (pwmValue > 220)
        {
          pwmValue = 220;
        }
      }

      statusCode = 0;
    }

    // Apply PWM to BOTH motor groups
    analogWrite(ENA1, pwmValue);
    analogWrite(ENA2, pwmValue);

    // --------------------------------------------------
    // SEND DATA TO RPI
    // --------------------------------------------------

    int rpmInt = (int)rpm;

    txMsg.can_id = 0x200;
    txMsg.can_dlc = 8;

    txMsg.data[0] = currentSignCode;
    txMsg.data[1] = speedLimitRPM & 0xFF;
    txMsg.data[2] = (speedLimitRPM >> 8) & 0xFF;
    txMsg.data[3] = rpmInt & 0xFF;
    txMsg.data[4] = (rpmInt >> 8) & 0xFF;
    txMsg.data[5] = pwmValue;
    txMsg.data[6] = statusCode;
    txMsg.data[7] = pulses & 0xFF;

    mcp2515.sendMessage(&txMsg);

    // --------------------------------------------------
    // SERIAL MONITOR
    // --------------------------------------------------

    Serial.println();
    Serial.println("----------- STATUS -----------");

    Serial.print("SIGN       : ");
    Serial.println(getSignName(currentSignCode));

    Serial.print("PULSES     : ");
    Serial.println(pulses);

    Serial.print("RPM        : ");
    Serial.println(rpm);

    Serial.print("RPM LIMIT  : ");
    Serial.println(speedLimitRPM);

    Serial.print("PWM        : ");
    Serial.println(pwmValue);

    Serial.print("STATUS     : ");
    Serial.println(statusCode);

    Serial.println("------------------------------");
  }
}

#----------------------------CAN LOG------------------------------
#CAN Receive data from stm to save in csv code in this file 

nano ~/speed_logs/can_receive_log.py 

#CAN Receive data watch live in this cmd

python3 ~/speed_logs/can_receive_log.py

#Actual CAN receive code in nano

import can
import csv
import os
from datetime import datetime

CSV_FILE = "/home/raspberrypi5/speed_logs/speed_log.csv"
IMAGE_URL = "http://10.1.233.89:8000/latest_detected.jpg"

os.makedirs("/home/raspberrypi5/speed_logs", exist_ok=True)

sign_names = {
    1: "Green Light",
    2: "Red Light",
    3: "Speed Limit 20",
    4: "Speed Limit 30",
    5: "Speed Limit 40",
    6: "Speed Limit 50",
    7: "Speed Limit 60",
    8: "Stop"
}

status_names = {
    0: "INACTIVE",
    1: "ACTIVE",
    2: "ACTIVE"
}

vehicle_status_names = {
    0: "NORMAL",
    1: "OVERSPEED",
    2: "STOP"
}

if not os.path.exists(CSV_FILE):
    with open(CSV_FILE, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([
            "timestamp",
            "traffic_sign",
            "vehicle_status",
            "vehicle_speed",
            "speed_reduction_status",
            "speed_limit_rpm",
            "image_url"
        ])

bus = can.interface.Bus(channel="can0", interface="socketcan")

print("CAN CSV Logger Started...")
print("Waiting for STM32 data on CAN ID 0x200...")

while True:
    msg = bus.recv()

    if msg is None:
        continue

    if msg.arbitration_id != 0x200:
        continue

    if len(msg.data) < 7:
        print("Incomplete CAN message:", msg)
        continue

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    sign_code = msg.data[0]

    speed_limit_rpm = msg.data[1] | (msg.data[2] << 8)
    vehicle_speed = msg.data[3] | (msg.data[4] << 8)
    pwm_value = msg.data[5]
    status_code = msg.data[6]

    traffic_sign = sign_names.get(sign_code, "Unknown")
    vehicle_status = vehicle_status_names.get(status_code, "UNKNOWN")
    speed_reduction_status = status_names.get(status_code, "UNKNOWN")

    with open(CSV_FILE, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([
            timestamp,
            traffic_sign,
            vehicle_status,
            vehicle_speed,
            speed_reduction_status,
            speed_limit_rpm,
            IMAGE_URL
        ])

    print(
        f"{timestamp} | "
        f"Sign={traffic_sign} | "
        f"Vehicle Speed={vehicle_speed} RPM | "
        f"Limit={speed_limit_rpm} RPM | "
        f"Status={vehicle_status} | "
        f"Reduction={speed_reduction_status} | "
        f"PWM={pwm_value}"
    )

# Auto start CAN logger code in this file

sudo nano /etc/systemd/system/can_logger.service

# Auto start CAN logger code

[Unit]
Description=CAN CSV Logger Auto Start
After=traffic_sign.service
Requires=traffic_sign.service

[Service]
User=raspberrypi5
WorkingDirectory=/home/raspberrypi5/speed_logs
ExecStart=/home/raspberrypi5/yolo-env/bin/python3 /home/raspberrypi5/speed_logs/can_receive_log.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target

#Check CAN logger service logs

journalctl -u can_logger.service -f

# CSV updates in this file

nano ~/speed_logs/speed_log.csv

#watch live updates of csv file by this cmd

python3 ~/speed_logs/speed_log.csv

#or by this cmd

tail -f /home/raspberrypi5/speed_logs/speed_log.csv

#------------------------DASHBOARD-----------------------------
#Dashboard code steps
 
source ~/yolo-env/bin/activate

cd ~/speed_logs

#Dashboard Code in this file 
nano dashboard.py

#Dashboard Code

import streamlit as st
import pandas as pd
import time
from pathlib import Path

CSV_FILE = Path("/home/raspberrypi5/speed_logs/speed_log.csv")
LIVE_IMAGE = Path("/home/raspberrypi5/speed_logs/images/latest_detected.jpg")

REQUIRED_COLUMNS = [
    "timestamp",
    "traffic_sign",
    "vehicle_speed",
    "vehicle_status",
    "speed_reduction_status",
    "speed_limit_rpm",
    "image_url"
]

st.set_page_config(
    page_title="Live Speed Compliance Dashboard",
    layout="wide"
)

st.title("Live Traffic Sign & Speed Compliance Dashboard")

placeholder = st.empty()

while True:
    with placeholder.container():

        if not CSV_FILE.exists():
            st.error("speed_log.csv not found")
            st.write(CSV_FILE)
            time.sleep(1)
            st.rerun()

        try:
            df = pd.read_csv(CSV_FILE)

            missing = [col for col in REQUIRED_COLUMNS if col not in df.columns]

            if missing:
                st.error("Wrong CSV header")
                st.write("Missing columns:", missing)
                st.code(",".join(REQUIRED_COLUMNS))
                time.sleep(1)
                st.rerun()

            if df.empty:
                st.warning("CSV is empty. Waiting for data...")
                time.sleep(1)
                st.rerun()

            df["vehicle_speed"] = pd.to_numeric(df["vehicle_speed"], errors="coerce")
            df["speed_limit_rpm"] = pd.to_numeric(df["speed_limit_rpm"], errors="coerce")
            df["timestamp"] = pd.to_datetime(df["timestamp"], errors="coerce")

            latest = df.iloc[-1]

            traffic_sign = str(latest["traffic_sign"])
            vehicle_speed = latest["vehicle_speed"]
            vehicle_status = str(latest["vehicle_status"])
            speed_reduction_status = str(latest["speed_reduction_status"])
            speed_limit_rpm = latest["speed_limit_rpm"]
            image_url = str(latest["image_url"]).strip()

            col1, col2, col3 = st.columns(3)

            col1.metric("Traffic Sign", traffic_sign)

            if pd.isna(vehicle_speed):
                col2.metric("Vehicle Speed (RPM)", "No RPM")
            else:
                col2.metric("Vehicle Speed (RPM)", int(vehicle_speed))

            if pd.isna(speed_limit_rpm):
                col3.metric("Speed Limit (RPM)", "No Limit")
            else:
                col3.metric("Speed Limit (RPM)", int(speed_limit_rpm))

            col4, col5 = st.columns(2)

            col4.metric("Vehicle Status", vehicle_status)
            col5.metric("Speed Reduction", speed_reduction_status)

            st.write("Last Update:", latest["timestamp"])

            st.divider()

            left, right = st.columns([2, 1])

            with left:
                st.subheader("Vehicle Speed vs Time")

                graph_df = df.dropna(subset=["timestamp"])
                graph_df = graph_df.set_index("timestamp")

                st.line_chart(graph_df[["vehicle_speed", "speed_limit_rpm"]])

                st.subheader("Recent Detection Log")
                st.dataframe(df.tail(10), use_container_width=True)

            with right:
                st.subheader("Live YOLO Detection Feed")

                if LIVE_IMAGE.exists():
                    st.image(
                        str(LIVE_IMAGE),
                        caption="Live YOLO Detection",
                        use_container_width=True
                    )
                else:
                    st.warning("latest_detected.jpg not found")
                    st.write(str(LIVE_IMAGE))
                    st.info("Your YOLO code must save frames with this exact name.")

                st.divider()

                st.subheader("Latest Detected Sign Image")

                if image_url.startswith("http"):
                    st.image(
                        image_url,
                        caption=traffic_sign,
                        use_container_width=True
                    )

                elif image_url and image_url.lower() != "nan" and Path(image_url).exists():
                    st.image(
                        image_url,
                        caption=traffic_sign,
                        use_container_width=True
                    )

                else:
                    st.warning("Image not found")
                    st.write(image_url)

        except Exception as e:
            st.error("Dashboard error")
            st.exception(e)

    time.sleep(1)
    st.rerun()

#Dashboard activate

source ~/yolo-env/bin/activate

streamlit run /home/raspberrypi5/speed_logs/dashboard.py --server.address 0.0.0.0

#If dashboard showing NaN for every parameter use these cmds 

cp /home/raspberrypi5/speed_logs/speed_log.csv /home/raspberrypi5/speed_logs/speed_log_backup.csv

echo "timestamp,traffic_sign,vehicle_status,vehicle_speed,speed_reduction_status,speed_limit_rpm,image_url" > /home/raspberrypi5/speed_logs/speed_log.csv

sudo systemctl restart can_logger.service

tail -f /home/raspberrypi5/speed_logs/speed_log.csv

streamlit run /home/raspberrypi5/speed_logs/dashboard.py --server.address 0.0.0.0

#If camera not found 
#power off rpi5
#attach cam module

rpicam-hello

#If network Down or socketbus not shut down properly or CAN window shut down 

sudo ip link set can0 down

sudo ip link set can0 type can bitrate 500000 restart-ms 100

sudo ip link set can0 up

ip -details link show can0 



