#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Adafruit_PWMServoDriver.h>
#include <esp32cam.h>
#include <Update.h>

// Servo Configuration
#define I2C_SDA 14 // GPIO14 (free pin on ESP32-CAM)
#define I2C_SCL 15 // GPIO15 (free pin on ESP32-CAM)
#define SERVOMIN 150
#define SERVOMAX 600
#define SERVO_FREQ 50
#define LED_GPIO_NUM 4 // LED Configuration (4 for flash LED or 33 for normal LED)

// WiFi Configuration
const char *ssid = "OPTIMUS";
const char *password = "qqwweeaaaa";

// Camera Configuration
static auto loRes = esp32cam::Resolution::find(320, 240);
static auto midRes = esp32cam::Resolution::find(350, 530);
static auto hiRes = esp32cam::Resolution::find(800, 600);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo ranges (min, max)
const int gripperOpen = 30;
const int gripperClosed = 99;
const int movementDelay = 20; // ms between steps
const int servoRanges[4][2] = {{1, 115}, {20, 80}, {11, 85}, {30, 99}};
const int homePosition[4] = {58, 23, 54, gripperClosed};
const int hazardousPickPosition[4] = {58, 60, 54, gripperOpen};
const int hazardousPlacePosition[4] = {6, 70, 29, gripperClosed};
const int nonRecyclablePickPosition[4] = {58, 60, 54, gripperOpen};
const int nonRecyclablePlacePosition[4] = {110, 70, 29, gripperClosed};
int servoAngles[4] = {58, 23, 54, 64}; // Midpoints

// Motion recording
struct MotionFrame
{
    int angles[4];
    unsigned long timeSinceLast;
};

const int maxFrames = 100;
MotionFrame motionSequence[maxFrames];

int recordedFrames = 0;
bool isRecording = false;
bool isPlaying = false;
unsigned long lastRecordTime = 0;
int currentPlayFrame = 0;
unsigned long playStartTime = 0;
bool isBusy = false;
String currentAction = "ready";

const char *updateHTML = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
        <title>ESP32-CAM Firmware Update</title>
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; }
            form { margin-top: 20px; }
            .progress { margin-top: 20px; width: 100%; background-color: #f3f3f3; }
            .progress-bar { width: 0%; height: 30px; background-color: #4CAF50; text-align: center; line-height: 30px; color: white; }
        </style>
    </head>
    <body>
        <h1>Firmware Update</h1>
        <form method='POST' action='/update' enctype='multipart/form-data'>
            <input type='file' name='update'>
            <input type='submit' value='Update'>
        </form>
        <div class="progress">
            <div class="progress-bar" id="progress">0%</div>
        </div>
        <script>
            document.querySelector('form').addEventListener('submit', function(e) {
                var form = this;
                var fileInput = form.querySelector('input[type="file"]');
                if (fileInput.files.length > 0) {
                    var xhr = new XMLHttpRequest();
                    var formData = new FormData(form);
                    
                    xhr.upload.addEventListener('progress', function(evt) {
                        if (evt.lengthComputable) {
                            var percentComplete = (evt.loaded / evt.total) * 100;
                            document.getElementById('progress').style.width = percentComplete + '%';
                            document.getElementById('progress').innerHTML = Math.round(percentComplete) + '%';
                        }
                    }, false);
                    
                    xhr.open('POST', '/update', true);
                    xhr.send(formData);
                }
                e.preventDefault();
            });
        </script>
    </body>
    </html>
    )rawliteral";

WebServer server(80);

//////////////////////////// Function Prototypes
void handleControl();
void handleSetPosition();
void handleRecord();
void handlePlay();
void handleClear();
void handleGetStatus();
void playMotion();
void setServoAngle(uint8_t servoNum, uint8_t angle);
void serveJpg();
void handleJpgLo();
void handleJpgHi();
void handleStream();
void handleSort();
void smoothMove(int targetAngles[4]);
void sortHazardous();
void sortNonRecyclable();
void handleUpload();
void handleUpdate();

//////////////////////////// Camera Functions
void serveJpg()
{
    auto frame = esp32cam::capture();
    if (frame == nullptr)
    {
        Serial.println("Capture Fail");
        server.send(503, "", "");
        return;
    }
    // Serial.printf("CAPTURE OK %dx%d %db\n", frame->getWidth(), frame->getHeight(),
    //               static_cast<int>(frame->size()));

    server.setContentLength(frame->size());
    server.send(200, "image/jpeg");
    WiFiClient client = server.client();
    frame->writeTo(client);
}

void handleJpgLo()
{
    if (!esp32cam::Camera.changeResolution(loRes))
    {
        Serial.println("SET-LO-RES FAIL");
    }
    serveJpg();
}

void handleJpgMid()
{
    if (!esp32cam::Camera.changeResolution(midRes))
    {
        Serial.println("SET-MID-RES FAIL");
    }
    serveJpg();
}

void handleJpgHi()
{
    if (!esp32cam::Camera.changeResolution(hiRes))
    {
        Serial.println("SET-HI-RES FAIL");
    }
    serveJpg();
}

void handleStream()
{
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/html",
                "<html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"></head>"
                "<body><img src=\"/cam-hi.jpg\" width=\"640\" height=\"480\"></body></html>");
}

//////////////////////////// Setup
void setup()
{
    Serial.begin(115200);

    // Initialize LED
    pinMode(LED_GPIO_NUM, OUTPUT);
    digitalWrite(LED_GPIO_NUM, LOW);

    // Initialize camera
    {
        using namespace esp32cam;
        Config cfg;
        cfg.setPins(pins::AiThinker);
        cfg.setResolution(hiRes);
        cfg.setBufferCount(2);
        cfg.setJpeg(80);

        bool ok = Camera.begin(cfg);
        Serial.println(ok ? "CAMERA OK" : "CAMERA FAIL");
        digitalWrite(LED_GPIO_NUM, ok);
        delay(100);
        digitalWrite(LED_GPIO_NUM, LOW);
    }

    // Connect to WiFi
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to WiFi! IP: ");
    Serial.println(WiFi.localIP());

    // Print camera URLs
    Serial.print("http://");
    Serial.print(WiFi.localIP());
    Serial.println("/cam-lo.jpg");
    Serial.print("http://");
    Serial.print(WiFi.localIP());
    Serial.println("/cam-hi.jpg");

    // API endpoints
    server.on("/api/control", HTTP_GET, handleControl);
    server.on("/api/setpos", HTTP_GET, handleSetPosition);
    server.on("/api/record", HTTP_GET, handleRecord);
    server.on("/api/play", HTTP_GET, handlePlay);
    server.on("/api/clear", HTTP_GET, handleClear);
    server.on("/api/status", HTTP_GET, handleGetStatus);
    server.on("/api/sort", HTTP_GET, handleSort);

    server.on("/update", HTTP_GET, []()
              {
        server.sendHeader("Connection", "close");
        server.send(200, "text/html", updateHTML); });

    server.on("/update", HTTP_POST, handleUpdate, handleUpload);
    // Camera endpoints
    server.on("/cam-lo.jpg", handleJpgLo);
    server.on("/cam-hi.jpg", handleJpgHi);
    server.on("/cam-mid.jpg", handleJpgMid);
    server.on("/stream", handleStream);

    // Enable CORS for localhost development
    server.onNotFound([]()
                      {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(404, "text/plain", "Not Found"); });

    // Initialize I2C with custom pins for ESP32-CAM
    Wire.begin(I2C_SDA, I2C_SCL);

    // Initialize PWM driver
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);

    // Initialize servos
    for (int i = 0; i < 4; i++)
        setServoAngle(i, homePosition[i]);

    server.begin();
}

//////////////////////////// Main Loop
void loop()
{
    server.handleClient();
    if (isPlaying)
        playMotion();
}

//////////////////////////// API Handlers
void handleControl()
{
    server.sendHeader("Access-Control-Allow-Origin", "*");
    if (server.hasArg("pos"))
    {
        String value = server.arg("pos");
        if (value == "home")
        {
            setServoAngle(0, 58);
            setServoAngle(1, 42);
            setServoAngle(2, 48);
            setServoAngle(3, 64);
        }
        else if (value == "led_on")
        {
            digitalWrite(LED_GPIO_NUM, HIGH);
        }
        else if (value == "led_off")
        {
            digitalWrite(LED_GPIO_NUM, LOW);
        }
    }
    server.send(200, "application/json", "{\"status\":\"OK\"}");
}

void handleSetPosition()
{
    server.sendHeader("Access-Control-Allow-Origin", "*");
    if (server.hasArg("servo") && server.hasArg("angle"))
    {
        int servoNum = server.arg("servo").toInt();
        int angle = server.arg("angle").toInt();

        if (servoNum >= 0 && servoNum < 4)
        {
            angle = constrain(angle, servoRanges[servoNum][0], servoRanges[servoNum][1]);
            setServoAngle(servoNum, angle);

            if (isRecording && recordedFrames < maxFrames)
            {
                unsigned long currentTime = millis();
                if (recordedFrames > 0)
                {
                    motionSequence[recordedFrames - 1].timeSinceLast = currentTime - lastRecordTime;
                }
                for (int i = 0; i < 4; i++)
                {
                    motionSequence[recordedFrames].angles[i] = servoAngles[i];
                }
                recordedFrames++;
                lastRecordTime = currentTime;
            }
        }
    }
    server.send(200, "application/json", "{\"status\":\"OK\"}");
}

void handleRecord()
{
    server.sendHeader("Access-Control-Allow-Origin", "*");
    isRecording = !isRecording;
    if (isRecording)
    {
        recordedFrames = 0;
        lastRecordTime = millis();
    }
    server.send(200, "application/json", "{\"status\":\"OK\",\"recording\":" + String(isRecording) + "}");
}

void handlePlay()
{
    server.sendHeader("Access-Control-Allow-Origin", "*");
    isPlaying = !isPlaying;
    if (isPlaying)
    {
        currentPlayFrame = 0;
        playStartTime = millis();
    }
    server.send(200, "application/json", "{\"status\":\"OK\",\"playing\":" + String(isPlaying) + "}");
}

void handleClear()
{
    server.sendHeader("Access-Control-Allow-Origin", "*");
    recordedFrames = 0;
    isRecording = false;
    isPlaying = false;
    server.send(200, "application/json", "{\"status\":\"OK\"}");
}

void handleGetStatus()
{
    server.sendHeader("Access-Control-Allow-Origin", "*");
    JsonDocument doc;
    doc["recording"] = isRecording;
    doc["playing"] = isPlaying;
    doc["frameCount"] = recordedFrames;
    doc["currentFrame"] = currentPlayFrame;
    doc["busy"] = isBusy;
    doc["action"] = currentAction;

    String json;
    serializeJson(doc, json);
    server.send(200, "application/json", json);
}
void playMotion()
{
    if (!isPlaying || currentPlayFrame >= recordedFrames)
    {
        isPlaying = false;
        return;
    }

    unsigned long currentTime = millis() - playStartTime;
    unsigned long frameTime = 0;

    for (int i = 0; i <= currentPlayFrame; i++)
    {
        frameTime += motionSequence[i].timeSinceLast;
    }

    if (currentTime >= frameTime)
    {
        for (int i = 0; i < 4; i++)
        {
            setServoAngle(i, motionSequence[currentPlayFrame].angles[i]);
        }
        currentPlayFrame++;
    }
}

void setServoAngle(uint8_t servoNum, uint8_t angle)
{
    angle = constrain(angle, servoRanges[servoNum][0], servoRanges[servoNum][1]);
    servoAngles[servoNum] = angle;
    uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(servoNum, 0, pulse);
    //  Serial.print("Servo ");
    // Serial.print(servoNum);
    // Serial.print(" set to angle: ");
    // Serial.print(angle);
    // Serial.print(" => pulse: ");
    // Serial.println(pulse);
}

void handleSort()
{
    server.sendHeader("Access-Control-Allow-Origin", "*");

    if (isBusy)
    {
        server.send(200, "application/json", "{\"status\":\"BUSY\"}");
        return;
    }

    if (!server.hasArg("type"))
    {
        server.send(400, "application/json", "{\"error\":\"Missing type parameter\"}");
        return;
    }

    String itemType = server.arg("type");
    isBusy = true;

    if (itemType == "hazardous")
    {
        currentAction = "sorting_hazardous";
        server.send(200, "application/json", "{\"status\":\"STARTED\"}");
        sortHazardous();
    }
    else if (itemType == "nonrecyclable")
    {
        currentAction = "sorting_nonrecyclable";
        server.send(200, "application/json", "{\"status\":\"STARTED\"}");
        sortNonRecyclable();
    }
    else
    {
        server.send(400, "application/json", "{\"error\":\"Invalid type\"}");
        isBusy = false;
        return;
    }
}

void smoothMove(int targetAngles[4])
{
    
    int steps = 0;
    int startAngles[4];

    // Calculate maximum steps needed
    for (int i = 1; i < 4; i++)
    {
        int diff = abs(targetAngles[i] - servoAngles[i]);
        if (diff > steps)
            steps = diff;
    }

    // Store starting positions
    memcpy(startAngles, servoAngles, sizeof(startAngles));

    // Execute movement
    for (int step = 0; step <= steps; step++)
    {
        for (int i = 1; i < 4; i++)
        {
            if (step <= abs(targetAngles[i] - startAngles[i]))
            {
                int direction = (targetAngles[i] > startAngles[i]) ? 1 : -1;
                setServoAngle(i, startAngles[i] + (step * direction));
            }
        }
        delay(movementDelay);
    }
}

void sortHazardous()
{
    // Move to pick position
    Serial.println("Sorting Hazardous");
    int pickPos[4];
    memcpy(pickPos, hazardousPickPosition, sizeof(pickPos));
    smoothMove(pickPos);
    
    // Open gripper
    setServoAngle(3, gripperOpen);
    delay(1000);
    
    // Close gripper (grab)
    setServoAngle(3, gripperClosed);
    delay(1000);
    
    setServoAngle(0,hazardousPlacePosition[0]);
    delay(1000);
    // Move to place position
    int placePos[4];
    memcpy(placePos, hazardousPlacePosition, sizeof(placePos));
    smoothMove(placePos);

    delay(1000);
    // Open gripper (release)
    setServoAngle(3, gripperOpen);
    delay(500);
    setServoAngle(3, gripperClosed);
    delay(1000);

    // Return home
    int home[4];
    memcpy(home, homePosition, sizeof(home));
    smoothMove(home);
    setServoAngle(0, homePosition[0]);
    delay(1000);

    // Close gripper
    setServoAngle(3, gripperClosed);

    isBusy = false;
    currentAction = "ready";
}

void sortNonRecyclable()
{
    Serial.println("Sorting NonRecyclable");
    // Move to pick position
    int pickPos[4];
    memcpy(pickPos, nonRecyclablePickPosition, sizeof(pickPos));
    smoothMove(pickPos);
    
    // Open gripper
    setServoAngle(3, gripperOpen);
    delay(1000);
    
    // Close gripper (grab)
    setServoAngle(3, gripperClosed);
    delay(1000);
    
    setServoAngle(0, nonRecyclablePlacePosition[0]);
    delay(1000);


    // Move to place position
    int placePos[4];
    memcpy(placePos, nonRecyclablePlacePosition, sizeof(placePos));
    smoothMove(placePos);

    delay(1000);
    // Open gripper (release)
    setServoAngle(3, gripperOpen);
    delay(500);
    setServoAngle(3, gripperClosed);
    delay(1000);

    // Return home
    int home[4];
    memcpy(home, homePosition, sizeof(home));
    smoothMove(home);

    setServoAngle(0, homePosition[0]);
    delay(1000);

    // Close gripper
    setServoAngle(3, gripperClosed);

    isBusy = false;
    currentAction = "ready";
}

void handleUpdate()
{
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
}

void handleUpload()
{
    HTTPUpload &upload = server.upload();
    if (upload.status == UPLOAD_FILE_START)
    {
        Serial.printf("Update: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN))
        { // Start with max available size
            Update.printError(Serial);
        }
    }
    else if (upload.status == UPLOAD_FILE_WRITE)
    {
        // Flashing firmware to ESP
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
        {
            Update.printError(Serial);
        }
    }
    else if (upload.status == UPLOAD_FILE_END)
    {
        if (Update.end(true))
        { // true to set the size to the current progress
            Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        }
        else
        {
            Update.printError(Serial);
        }
    }
}