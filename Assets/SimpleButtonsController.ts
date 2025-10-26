import { InteractorEvent } from "SpectaclesInteractionKit.lspkg/Core/Interactor/InteractorEvent";
import { SIK } from "SpectaclesInteractionKit.lspkg/SIK";
import NativeLogger from "SpectaclesInteractionKit.lspkg/Utils/NativeLogger";
import { Interactable } from "SpectaclesInteractionKit.lspkg/Components/Interaction/Interactable/Interactable";
import { RectangleButton } from "SpectaclesUIKit.lspkg/Scripts/Components/Button/RectangleButton";

const log = new NativeLogger("ButtonControllerLogger");

/**
 * SimpleButtonsController
 * -----------------------
 * Sends button interaction events (up, down, left, right)
 * from the Spectacles HUD to a WebSocket server.
 *
 * Compatible with the Python server running on ws://<your-ip>:8765
 */
@component
export class SimpleButtonsController extends BaseScriptComponent {

  // WebSocket connection
  private webSocket: WebSocket | null = null;

  // Change this to your laptop’s local IP (same Wi-Fi network)
  // Example: ws://192.168.1.25:8765
  private readonly WEBSOCKET_URL = "ws://192.168.0.64:8765";

  @input @allowUndefined internetModule: InternetModule;
  @input @allowUndefined buttonUp: RectangleButton;
  @input @allowUndefined buttonDown: RectangleButton;
  @input @allowUndefined buttonLeft: RectangleButton;
  @input @allowUndefined buttonRight: RectangleButton;

  // --------------------------- Lifecycle ---------------------------

  onAwake() {
    this.initializeWebSocket();
    this.createEvent("OnStartEvent").bind(() => {
      this.bindButtonEvents();
      log.d("OnStart event triggered");
      print("OnStart event triggered");
    });
  }

  onDestroy() {
    if (this.webSocket) {
      this.webSocket.close();
      print("WebSocket connection closed");
      log.d("WebSocket connection closed");
    }
  }

  // --------------------------- WebSocket Setup ---------------------------

  private initializeWebSocket() {
    if (!this.internetModule) {
      print("Error: InternetModule not assigned!");
      log.e("InternetModule not assigned");
      return;
    }

    try {
      this.webSocket = this.internetModule.createWebSocket(this.WEBSOCKET_URL);

      this.webSocket.onopen = (event) => {
        print("✅ WebSocket connected to server!");
        log.d("WebSocket connection established");
      };

      this.webSocket.onmessage = (event) => {
        print("📩 Message from server: " + event.data);
        log.d("Received message: " + event.data);
      };

      this.webSocket.onclose = (event) => {
        print(`⚠️ WebSocket closed (code ${event.code})`);
        log.d(`WebSocket closed: ${event.code}`);
      };

      this.webSocket.onerror = (event) => {
        print("❌ WebSocket error occurred");
        log.e("WebSocket error occurred");
      };
    } catch (error) {
      print("Failed to create WebSocket: " + error);
      log.e("Failed to create WebSocket: " + error);
    }
  }

  // --------------------------- Data Sending ---------------------------

  private sendButtonData(buttonName: string, action: string) {
    if (!this.webSocket) {
      print("⚠️ WebSocket not initialized. Reconnecting...");
      this.initializeWebSocket();
      return;
    }

    if (this.webSocket.readyState !== 1) { // 1 = OPEN
      print(`⚠️ WebSocket not ready (state ${this.webSocket.readyState}). Reconnecting...`);
      this.initializeWebSocket();
      return;
    }

    const buttonData = {
      type: "button",
      button: buttonName,
      action: action,
      timestamp: Date.now()
    };

    try {
      this.webSocket.send(JSON.stringify(buttonData));
      print(`📤 Sent button data: ${buttonName} - ${action}`);
      log.d(`Sent button data: ${buttonName} - ${action}`);
    } catch (error) {
      print("Failed to send button data: " + error);
      log.e("Failed to send button data: " + error);
    }
  }

  // --------------------------- Button Event Bindings ---------------------------

  private bindButtonEvents() {
    if (!this.buttonUp || !this.buttonDown || !this.buttonLeft || !this.buttonRight) {
      print("❌ One or more buttons are not assigned in the Inspector!");
      return;
    }

    // UP
    this.buttonUp.onTriggerDown.add(() => this.sendButtonData("up", "triggerDown"));
    this.buttonUp.onTriggerUp.add(() => this.sendButtonData("up", "triggerUp"));

    // DOWN
    this.buttonDown.onTriggerDown.add(() => this.sendButtonData("down", "triggerDown"));
    this.buttonDown.onTriggerUp.add(() => this.sendButtonData("down", "triggerUp"));

    // LEFT
    this.buttonLeft.onTriggerDown.add(() => this.sendButtonData("left", "triggerDown"));
    this.buttonLeft.onTriggerUp.add(() => this.sendButtonData("left", "triggerUp"));

    // RIGHT
    this.buttonRight.onTriggerDown.add(() => this.sendButtonData("right", "triggerDown"));
    this.buttonRight.onTriggerUp.add(() => this.sendButtonData("right", "triggerUp"));

    print("✅ Button event bindings completed!");
    log.d("Button event bindings completed");
  }
}
