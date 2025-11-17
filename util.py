# util.py
class sensors:
    @staticmethod
    def get_sensor_value(port, mode, index):
        # Fake sensor value (just for testing on PC)
        print(f"[util.sensors] get_sensor_value({port}, {mode}, {index})")
        return 0

class time:
    current_time = 0

    @staticmethod
    def get_time():
        time.current_time += 1
        return time.current_time

    @staticmethod
    def start_time():
        time.current_time = 0

    @staticmethod
    def stop_time():
        pass

    @staticmethod
    def reset_time():
        time.current_time = 0

class scratch:
    @staticmethod
    def convert_image(image_str, brightness):
        # Return something resembling the image data
        return f"[scratch image {image_str} @ brightness {brightness}]"
