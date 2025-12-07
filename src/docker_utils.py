import subprocess
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def check_docker():
    """
    Checks if Docker daemon is running and accessible.
    Returns True if Docker is running, False otherwise.
    """
    try:
        subprocess.run(['docker', 'info'], check=True, capture_output=True)
        logging.info("Docker daemon is running.")
        return True
    except subprocess.CalledProcessError as e:
        logging.error(f"Docker daemon not running or accessible: {e.stderr.decode().strip()}")
        return False
    except FileNotFoundError:
        logging.error("Docker command not found. Is Docker installed and in your PATH?")
        return False

def check_docker_image(image_name):
    """
    Checks if a specific Docker image exists locally.
    Returns True if the image exists, False otherwise.
    """
    if not check_docker():
        return False

    try:
        subprocess.run(['docker', 'image', 'inspect', image_name], check=True, capture_output=True)
        logging.info(f"Docker image '{image_name}' found locally.")
        return True
    except subprocess.CalledProcessError:
        logging.warning(f"Docker image '{image_name}' not found locally.")
        return False

def check_docker_access():
    """
    Checks if the current user has permission to interact with Docker.
    Returns True if access is granted, False otherwise.
    """
    try:
        # Try to run a simple command that requires Docker access
        subprocess.run(['docker', 'ps'], check=True, capture_output=True)
        logging.info("Docker access granted for current user.")
        return True
    except subprocess.CalledProcessError as e:
        logging.error(f"Docker access denied for current user. Error: {e.stderr.decode().strip()}")
        logging.error("Consider adding your user to the 'docker' group: 'sudo usermod -aG docker $USER && newgrp docker'")
        return False
    except FileNotFoundError:
        # This case is already covered by check_docker, but added for robustness
        logging.error("Docker command not found. Is Docker installed and in your PATH?")
        return False

if __name__ == '__main__':
    print("--- Docker Status Check ---")
    if check_docker():
        print("Docker is running.")
        if check_docker_access():
            print("You have Docker access.")
        else:
            print("You DO NOT have Docker access.")
            print("Please follow the instructions above to fix it.")
    else:
        print("Docker is NOT running.")
    
    print("\n--- Docker Image Check (example: 'hello-world') ---")
    if check_docker_image('hello-world'):
        print("Image 'hello-world' exists.")
    else:
        print("Image 'hello-world' does not exist. Try 'docker pull hello-world'.")
