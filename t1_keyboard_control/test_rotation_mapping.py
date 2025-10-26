#!/usr/bin/env python3
"""
Test script to demonstrate the new rotation mapping functionality
This shows how rotateBy values map to Q/E commands
"""

import json
import asyncio

def test_rotation_mapping():
    """Test the rotation mapping logic"""
    
    # Test cases with different rotateBy values
    test_cases = [
        {"rotateBy": 45.0, "expected": "e", "description": "Small right rotation"},
        {"rotateBy": -60.0, "expected": "q", "description": "Medium left rotation"}, 
        {"rotateBy": 120.0, "expected": "e", "description": "Large right rotation"},
        {"rotateBy": -150.0, "expected": "q", "description": "Large left rotation"},
        {"rotateBy": 0.0, "expected": None, "description": "No rotation"},
        {"rotateBy": 15.0, "expected": "e", "description": "Small right rotation"},
        {"rotateBy": -30.0, "expected": "q", "description": "Small left rotation"}
    ]
    
    print("🧪 Testing Rotation Mapping Logic")
    print("=" * 50)
    
    for test in test_cases:
        rotate_by = test["rotateBy"]
        expected = test["expected"]
        
        # Apply the same logic as in the WebSocket handler
        if rotate_by != 0:
            if rotate_by > 0:  # Positive rotation = turn right
                command_key = 'e'  # Turn right
            else:  # Negative rotation = turn left
                command_key = 'q'  # Turn left
            
            # Calculate repeat count (short intervals for larger rotations)
            rotation_intensity = min(abs(rotate_by) / 30, 5)
            repeat_count = max(1, int(rotation_intensity))
        else:
            command_key = None
            repeat_count = 0
        
        # Check if mapping is correct
        is_correct = command_key == expected
        status = "✅" if is_correct else "❌"
        
        print(f"{status} {test['description']}")
        print(f"   Input: rotateBy={rotate_by}")
        print(f"   Expected: {expected}, Got: {command_key}")
        print(f"   Repeat count: {repeat_count}")
        print()

def test_json_cleaning():
    """Test the JSON cleaning functionality"""
    
    print("🧼 Testing JSON Message Cleaning")
    print("=" * 50)
    
    # Test cases with problematic JSON messages from your logs
    test_messages = [
        '{"type":"button","button":"turn_right","action":"triggerDown","rotateBy":1.500008603866604,"timestamp":1761457439328}#',
        '{"type":"button","button":"turn_right","action":"triggerDown","rotateBy":1.499978747686555,"timestamp":1761457439515}#',
        '{"type":"button","button":"turn_right","action":"triggerUp","rotateBy":0,"timestamp":1761457439581}7',
        '{"type":"button","button":"turn_left","action":"triggerDown","rotateBy":-1.2,"timestamp":1761457439581}123'
    ]
    
    for i, message in enumerate(test_messages, 1):
        print(f"Test {i}:")
        print(f"  Original: {message}")
        
        # Apply the cleaning logic
        clean_message = message.rstrip('#0123456789')
        print(f"  Cleaned:  {clean_message}")
        
        try:
            data = json.loads(clean_message)
            rotate_by = data.get("rotateBy", 0)
            button = data.get("button")
            action = data.get("action")
            print(f"  ✅ Parsed successfully: button={button}, action={action}, rotateBy={rotate_by}")
        except json.JSONDecodeError as e:
            print(f"  ❌ JSON parsing failed: {e}")
        print()

if __name__ == "__main__":
    test_rotation_mapping()
    test_json_cleaning()
    
    print("🎯 Summary of Changes")
    print("=" * 50)
    print("✅ Added rotateBy parameter extraction from WebSocket messages")
    print("✅ Fixed JSON parsing by cleaning trailing characters (#, numbers)")
    print("✅ Mapped positive rotateBy values (+) to 'e' key (turn right)")
    print("✅ Mapped negative rotateBy values (-) to 'q' key (turn left)")
    print("✅ Added short intervals: larger rotations repeat commands more times")
    print("✅ Rotation intensity scaling: rotateBy/30 determines repeat count")
    print()
    print("🎮 Usage Examples:")
    print("  - rotateBy: +90  → 'e' command × 3 times (right turn)")
    print("  - rotateBy: -45  → 'q' command × 1 time (left turn)")
    print("  - rotateBy: +150 → 'e' command × 5 times (strong right turn)")
    print("  - rotateBy: 0    → No rotation command")