// @input Asset.Material videoMaterial
// @input InternetModule internetModule  
// @input string serverUrl = "ws://192.168.1.100:8080"
// @input bool enableDebugLogs = true
// @input Component.Image image

@component
export class RobotFeedDisplay extends BaseScriptComponent {

  @input videoMaterial?: Material;
  @input internetModule?: InternetModule;
  @input serverUrl: string = "ws://192.168.1.100:8080";
  @input enableDebugLogs: boolean = true;
  @input image: Image;

  private webSocket: WebSocket | null = null;
  private textureProvider: ProceduralTextureProvider | null = null;
  private frameCount = 0;
  private lastLogTime = 0;
  private reconnectAttempts = 0;
  private readonly maxReconnectAttempts = 10;
  private isConnecting = false;
  private lastFrameTime = 0;

  onAwake() {
    this.log("🎥 Robot Feed Display Starting...");
    if (this.validateInputs()) {
      this.initializeTexture();
      this.initializeWebSocket();
      this.createEvent("UpdateEvent").bind(() => this.onUpdate());
    }
  }

  private validateInputs(): boolean {
    if (!this.internetModule) return this.logAndReturn("❌ InternetModule not assigned!");
    if (!this.videoMaterial && !this.image) return this.logAndReturn("❌ No material or image assigned!");
    if (!this.serverUrl || this.serverUrl.trim() === "") return this.logAndReturn("❌ Server URL not set!");
    this.log(`✅ Inputs validated. Connecting to: ${this.serverUrl}`);
    return true;
  }

  private logAndReturn(msg: string): boolean {
    this.log(msg);
    return false;
  }

  private initializeTexture() {
    const tex = ProceduralTextureProvider.create(512, 512, Colorspace.RGBA);
    this.textureProvider = tex.control as ProceduralTextureProvider;

    if (this.videoMaterial) {
      this.videoMaterial.mainPass.baseTex = tex;
    }
    if (this.image) {
      this.image.mainPass.baseTex = tex;
    }
  }

  private initializeWebSocket() {
    if (this.isConnecting) return;
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      this.log("❌ Max reconnect attempts reached");
      return;
    }

    this.isConnecting = true;
    this.log(`🔌 Connecting to ${this.serverUrl} (attempt ${this.reconnectAttempts + 1})`);

    try {
      this.webSocket = this.internetModule!.createWebSocket(this.serverUrl);

      this.webSocket.onopen = () => {
        this.log("✅ Connected to robot camera!");
        this.reconnectAttempts = 0;
        this.isConnecting = false;
      };

      this.webSocket.onmessage = (event) => this.handleMessage(event);

      this.webSocket.onclose = (e) => {
        this.isConnecting = false;
        this.log(`⚠️ Connection closed (code ${e.code})`);
        this.scheduleReconnect();
      };

      this.webSocket.onerror = () => {
        this.isConnecting = false;
        this.log("❌ WebSocket error");
        this.scheduleReconnect();
      };

    } catch (err) {
      this.isConnecting = false;
      this.log("❌ Failed to create WebSocket: " + err);
      this.scheduleReconnect();
    }
  }

  private handleMessage(event: any) {
    try {
      if (typeof event.data === "string") {
        this.updateTexture(event.data.trim());
      } else {
        this.log("⚠️ Unexpected message type: " + typeof event.data);
      }
    } catch (e) {
      this.log("❌ Error handling message: " + e);
    }
  }

  private updateTexture(base64Data: string) {
    if (!this.textureProvider) return;
  
    const bgr = base64ToUint8Array(base64Data);
    const pixelCount = bgr.length / 3;
    const rgba = new Uint8Array(pixelCount * 4);
  
    for (let i = 0, j = 0; i < bgr.length; i += 3, j += 4) {
      rgba[j] = bgr[i + 2];     // R ← B
      rgba[j + 1] = bgr[i + 1]; // G ← G
      rgba[j + 2] = bgr[i];     // B ← R
      rgba[j + 3] = 255;        // A ← opaque
    }
  
    const expectedSize = 512 * 512 * 4;
    if (rgba.length < expectedSize) {
      this.log(`⚠️ Skipped incomplete frame (${rgba.length}/${expectedSize})`);
      return;
    }
  
    this.textureProvider.setPixels(0, 0, 512, 512, rgba);
  }
  
  

  private scheduleReconnect() {
    this.reconnectAttempts++;
    const delay = Math.min(2.0 * this.reconnectAttempts, 10.0);
    this.log(`🔁 Reconnecting in ${delay}s...`);

    const evt = this.createEvent("DelayedCallbackEvent");
    evt.bind(() => this.initializeWebSocket());
    evt.reset(delay);
  }

  private onUpdate() {
    const now = getTime();

    if (now - this.lastLogTime > 5.0 && this.frameCount > 0) {
      const fps = this.frameCount / 5.0;
      this.log(`📊 FPS: ${fps.toFixed(1)}`);
      this.frameCount = 0;
      this.lastLogTime = now;
    }

    if (this.webSocket && this.lastFrameTime > 0 && now - this.lastFrameTime > 10.0) {
      this.log("⚠️ No frames in 10s, reconnecting...");
      this.closeConnection();
      this.scheduleReconnect();
    }
  }

  private closeConnection() {
    if (this.webSocket) {
      try { this.webSocket.close(); } catch {}
      this.webSocket = null;
    }
  }

  private log(message: string) {
    if (this.enableDebugLogs) print(`[RobotFeed] ${message}`);
  }

  onDestroy() {
    this.log("🛑 Destroying RobotFeedDisplay...");
    this.closeConnection();
  }
}

/**
 * Convert Base64 string → Uint8Array
 */
function base64ToUint8Array(base64: string): Uint8Array {
    const chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    const lookup = new Uint8Array(256);
    for (let i = 0; i < chars.length; i++) lookup[chars.charCodeAt(i)] = i;
  
    let bufferLength = base64.length * 0.75;
    if (base64.endsWith("==")) bufferLength -= 2;
    else if (base64.endsWith("=")) bufferLength -= 1;
  
    const bytes = new Uint8Array(bufferLength);
    let p = 0;
  
    for (let i = 0; i < base64.length; i += 4) {
      const encoded1 = lookup[base64.charCodeAt(i)];
      const encoded2 = lookup[base64.charCodeAt(i + 1)];
      const encoded3 = lookup[base64.charCodeAt(i + 2)];
      const encoded4 = lookup[base64.charCodeAt(i + 3)];
      bytes[p++] = (encoded1 << 2) | (encoded2 >> 4);
      bytes[p++] = ((encoded2 & 15) << 4) | (encoded3 >> 2);
      bytes[p++] = ((encoded3 & 3) << 6) | (encoded4 & 63);
    }
  
    return bytes;
  }
  