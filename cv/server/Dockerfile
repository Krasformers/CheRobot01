FROM ubuntu:latest
RUN apt-get -y update && apt-get -y full-upgrade
WORKDIR /app
COPY * ./
RUN apt-get -y install $(awk '{print $1}' package.txt) 
RUN python3 -m pip install -r requirements.txt --break-system-packages --no-cache-dir --root-user-action=ignore
CMD ["python3", "main.py"]