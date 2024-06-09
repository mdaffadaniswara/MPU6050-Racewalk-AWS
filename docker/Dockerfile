FROM python:3.10.4

WORKDIR /app
COPY requirement.txt .
RUN pip3 install --no-cache-dir -r requirement.txt

COPY . .

ENTRYPOINT ["gunicorn", "-b", "0.0.0.0:8080", "app:app", "-n"]