import os
from pprint import pprint
from flask import Flask, request
from werkzeug.middleware.proxy_fix import ProxyFix
from src.inference import predict
import joblib

app = Flask(__name__)
model = joblib.load('./src/model/svm_model.pkl')

# Since the web application runs behind a proxy (nginx), we need to
# add this setting to our app.
app.wsgi_app = ProxyFix(app.wsgi_app, x_for=1, x_proto=1, x_host=1, x_prefix=1)

@app.route("/ping", methods=["GET"])
def ping():
    """
    Healthcheck function.
    """
    return "pong"

@app.route("/invocations", methods=["POST"])
def invocations():
    """
    Function which responds to the invocations requests.
    """
    body = request.json
    return predict(body=body, model=model)