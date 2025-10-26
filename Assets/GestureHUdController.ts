//@input Asset.InternetModule internetModule
//@input Component.Text statusText
//@input Component.Text instructionText

@component
export class GestureHUDController extends BaseScriptComponent {
    // Input properties (declared via @input comments above)
    public internetModule: InternetModule;
    public statusText: Text;
    public instructionText: Text;
    
    // WebSocket connection
    private webSocket: WebSocket | null = null;
    
    // Gesture state tracking
    private isPinching: boolean = false;
    private pinchStartPosition: vec3 = new vec3(0, 0, 0);
    private lastGestureDirection: string = "none";
    
    // Configuration
    private readonly WEBSOCKET_URL = "wss://echo.websocket.org"; // Replace with your WebSocket server
    private readonly MIN_GESTURE_DISTANCE = 0.1; // Minimum distance for directional gesture
    
    onAwake() {
        this.initializeWebSocket();
        this.setupGestureDetection();
        this.updateStatusText("Initializing...");
        this.updateInstructionText("Pinch and move in any direction to send gesture data");
    }
    
    private initializeWebSocket() {
        if (!this.internetModule) {
            print("Error: InternetModule not assigned!");
            this.updateStatusText("Error: No InternetModule");
            return;
        }
        
        try {
            this.webSocket = this.internetModule.createWebSocket(this.WEBSOCKET_URL);
            this.webSocket.binaryType = "blob";
            
            this.webSocket.onopen = (event) => {
                print("WebSocket connected!");
                this.updateStatusText("Connected to WebSocket");
            };
            
            this.webSocket.onmessage = async (event) => {
                if (event.data instanceof Blob) {
                    const text = await event.data.text();
                    print("Received: " + text);
                } else {
                    print("Received: " + event.data);
                }
            };
            
            this.webSocket.onclose = (event) => {
                print("WebSocket closed: " + event.code);
                this.updateStatusText("WebSocket disconnected");
            };
            
            this.webSocket.onerror = (event) => {
                print("WebSocket error");
                this.updateStatusText("WebSocket error");
            };
            
        } catch (error) {
            print("Failed to create WebSocket: " + error);
            this.updateStatusText("WebSocket connection failed");
        }
    }
    
    private setupGestureDetection() {
        // Get the Spectacles Interaction Kit API
        const sikAPI = (global as any).SIK?.SIKAPI;
        if (!sikAPI) {
            print("Error: SIK API not available!");
            this.updateStatusText("Error: SIK not available");
            return;
        }
        
        // Set up hand tracking for both hands
        try {
            // Right hand pinch detection
            const rightHand = sikAPI.getHand("right");
            if (rightHand) {
                rightHand.onPinchDown.add(() => {
                    this.onPinchStart("right");
                });
                
                rightHand.onPinchUp.add(() => {
                    this.onPinchEnd("right");
                });
            }
            
            // Left hand pinch detection
            const leftHand = sikAPI.getHand("left");
            if (leftHand) {
                leftHand.onPinchDown.add(() => {
                    this.onPinchStart("left");
                });
                
                leftHand.onPinchUp.add(() => {
                    this.onPinchEnd("left");
                });
            }
            
            print("Gesture detection setup complete");
            
        } catch (error) {
            print("Error setting up gesture detection: " + error);
            this.updateStatusText("Gesture setup failed");
        }
    }
    
    private onPinchStart(handType: string) {
        this.isPinching = true;
        print(`Pinch started with ${handType} hand`);
        
        // Get hand position for gesture tracking
        const sikAPI = (global as any).SIK?.SIKAPI;
        if (sikAPI) {
            const hand = sikAPI.getHand(handType);
            if (hand && hand.isTracked()) {
                // Get index finger tip position as reference
                const indexTip = hand.getJoint("index_tip");
                if (indexTip) {
                    this.pinchStartPosition = indexTip.getWorldPosition();
                }
            }
        }
        
        this.updateStatusText(`Pinching with ${handType} hand`);
    }
    
    private onPinchEnd(handType: string) {
        if (!this.isPinching) return;
        
        this.isPinching = false;
        print(`Pinch ended with ${handType} hand`);
        
        // Calculate gesture direction
        const gestureDirection = this.calculateGestureDirection(handType);
        this.sendGestureData(handType, gestureDirection);
        
        this.updateStatusText(`Gesture: ${gestureDirection}`);
    }
    
    private calculateGestureDirection(handType: string): string {
        const sikAPI = (global as any).SIK?.SIKAPI;
        if (!sikAPI) return "none";
        
        const hand = sikAPI.getHand(handType);
        if (!hand || !hand.isTracked()) return "none";
        
        const indexTip = hand.getJoint("index_tip");
        if (!indexTip) return "none";
        
        const currentPosition = indexTip.getWorldPosition();
        const delta = currentPosition.sub(this.pinchStartPosition);
        
        // Calculate the dominant direction
        const absX = Math.abs(delta.x);
        const absY = Math.abs(delta.y);
        const absZ = Math.abs(delta.z);
        
        // Check if movement is significant enough
        if (absX < this.MIN_GESTURE_DISTANCE && absY < this.MIN_GESTURE_DISTANCE && absZ < this.MIN_GESTURE_DISTANCE) {
            return "tap"; // Just a tap, no significant movement
        }
        
        // Determine primary direction
        if (absX > absY && absX > absZ) {
            return delta.x > 0 ? "right" : "left";
        } else if (absY > absX && absY > absZ) {
            return delta.y > 0 ? "up" : "down";
        } else if (absZ > absX && absZ > absY) {
            return delta.z > 0 ? "forward" : "backward";
        }
        
        return "none";
    }
    
    private sendGestureData(handType: string, direction: string) {
        if (!this.webSocket || this.webSocket.readyState !== 1) { // 1 = OPEN
            print("WebSocket not connected, cannot send data");
            return;
        }
        
        const gestureData = {
            type: "gesture",
            hand: handType,
            direction: direction,
            timestamp: Date.now(),
            position: {
                x: this.pinchStartPosition.x,
                y: this.pinchStartPosition.y,
                z: this.pinchStartPosition.z
            }
        };
        
        try {
            this.webSocket.send(JSON.stringify(gestureData));
            print(`Sent gesture: ${handType} hand, direction: ${direction}`);
        } catch (error) {
            print("Failed to send gesture data: " + error);
        }
    }
    
    private updateStatusText(message: string) {
        if (this.statusText) {
            this.statusText.text = `Status: ${message}`;
        }
    }
    
    private updateInstructionText(message: string) {
        if (this.instructionText) {
            this.instructionText.text = message;
        }
    }
    
    onDestroy() {
        if (this.webSocket) {
            this.webSocket.close();
        }
    }
}
