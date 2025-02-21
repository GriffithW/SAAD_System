import time
from multiprocessing import Manager, Process
from web_server import run_server

def worker_task(shared_data):
    counter = 0
    while True:
        # Periodically update the shared data
        counter += 1
        print(f"Worker updated message: {shared_data['message']}")
        time.sleep(5)

if __name__ == "__main__":
    # Initialize the Manager and shared dictionary
    manager = Manager()
    shared_data = manager.dict()
    shared_data["message"] = "Worker starting up..."
    shared_data["target_coordinates"] = "NA"

    # Start worker and web server processes
    worker_process = Process(target=worker_task, args=(shared_data,))
    server_process = Process(target=run_server, args=(shared_data,))

    worker_process.start()
    server_process.start()

    # Wait for processes to finish
    worker_process.join()
    server_process.join()
