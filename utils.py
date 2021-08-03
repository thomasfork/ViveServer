import subprocess


def get_hostname():
    hostname = subprocess.run(['hostname', '-I'], capture_output = True).stdout.decode().split(' ')[0]
    return hostname

if __name__ == '__main__':
    get_hostname()
