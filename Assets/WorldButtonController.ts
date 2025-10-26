@component
export class WorldButtonController extends BaseScriptComponent {

  @input myButton: SceneObject;
  @input internetModule: InternetModule;
  @input cameraObject: SceneObject;

  private webSocket: WebSocket | null = null;
  private readonly WEBSOCKET_URL = "ws://192.168.0.64:8765";
  private reconnectInterval = 2000;

  private lastAngle = 0;
  private turningState: "none" | "left" | "right" = "none";

  onAwake() {
    print("🚀 WorldButtonController Awake");
    this.initializeWebSocket();
    this.createEvent("UpdateEvent").bind(() => this.onUpdateFrame());
  }

  private initializeWebSocket() {
    if (!this.internetModule) {
      print("❌ InternetModule not assigned!");
      return;
    }

    try {
      this.webSocket = this.internetModule.createWebSocket(this.WEBSOCKET_URL);
      this.webSocket.onopen = () => print("✅ Connected to rotation server!");
      this.webSocket.onclose = (e) => {
        print(`⚠️ WebSocket closed (code ${e.code}). Reconnecting...`);
        this.scheduleReconnect();
      };
      this.webSocket.onerror = () => {
        print("❌ WebSocket error");
        this.scheduleReconnect();
      };
    } catch (err) {
      print("❌ Failed to create WebSocket: " + err);
      this.scheduleReconnect();
    }
  }

  private scheduleReconnect() {
    const evt = this.createEvent("DelayedCallbackEvent");
    evt.bind(() => {
      print("🔁 Attempting reconnect...");
      this.initializeWebSocket();
    });
    evt.reset(this.reconnectInterval / 1000);
  }

  private onUpdateFrame() {
    if (!this.cameraObject || !this.myButton) return;

    const camTransform = this.cameraObject.getTransform();
    const btnTransform = this.myButton.getTransform();

    const camPos = camTransform.getWorldPosition();
    const buttonPos = btnTransform.getWorldPosition();

    const toButton = buttonPos.sub(camPos).normalize();
    const forward = camTransform.forward.normalize();

    const angleRad = Math.atan2(
      toButton.x * forward.z - toButton.z * forward.x,
      toButton.x * forward.x + toButton.z * forward.z
    );
    let yawDeg = angleRad * (180 / Math.PI);
    if (yawDeg < 0) yawDeg += 360;

    const delta = yawDeg - this.lastAngle;
    this.lastAngle = yawDeg;

    // Filter jitter
    if (Math.abs(delta) < 1) {
      if (this.turningState !== "none") {
        this.sendButtonEvent(
          this.turningState === "left" ? "turn_left" : "turn_right",
          "triggerUp",
          0
        );
        this.turningState = "none";
      }
      return;
    }

    const newDirection = delta > 0 ? "right" : "left";
    const newButton = newDirection === "right" ? "turn_right" : "turn_left";

    if (this.turningState !== newDirection) {
      if (this.turningState !== "none") {
        const prevButton =
          this.turningState === "left" ? "turn_left" : "turn_right";
        this.sendButtonEvent(prevButton, "triggerUp", 0);
      }
      this.sendButtonEvent(newButton, "triggerDown", delta);
      this.turningState = newDirection;
    } else {
      // Continuously send delta while turning
      this.sendButtonEvent(newButton, "triggerDown", delta);
    }

    print(`📏 Turning ${newDirection.toUpperCase()} (Δ ${delta.toFixed(2)}°)`);
  }

  private sendButtonEvent(button: string, action: string, deltaAngle?: number) {
    if (!this.webSocket || this.webSocket.readyState !== 1) return;

    const buttonData = {
      type: "button",
      button: button,
      action: action,
      rotateBy: deltaAngle ?? 0,
      timestamp: Date.now(),
    };

    try {
      this.webSocket.send(JSON.stringify(buttonData));
      print(`📤 Sent: ${button} - ${action} (Δ ${deltaAngle?.toFixed?.(2) ?? "0"}°)`);
    } catch (err) {
      print("⚠️ Failed to send rotation: " + err);
    }
  }
}
