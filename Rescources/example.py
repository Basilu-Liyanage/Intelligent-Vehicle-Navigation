def eye_to_steering(eye_angle):
    # Main formula
    if eye_angle <= 125:
        driver_angle = 35 + (125 - eye_angle) / 3
    else:
        driver_angle = 35 - (eye_angle - 125) * 7 / 15
    
    # Keep the result between 0 and 60
    driver_angle = max(0, min(60, driver_angle))
    
    return round(driver_angle, 2)  # Round to 2 decimal places


# === How to use it ===
if __name__ == "__main__":
    print("Eye to Driver Steering Converter")
    print("==============================")
    
    while True:
        try:
            eye = float(input("\nEnter Eye Angle (or type 'q' to quit): "))
            steering = eye_to_steering(eye)
            print(f"Eye Angle: {eye}°  →  Driver Steering: {steering}°")
        except ValueError:
            print("Goodbye!")
            break