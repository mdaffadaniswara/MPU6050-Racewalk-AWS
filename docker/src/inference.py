import pandas as pd

def predict(body: dict, model) -> dict:
    """
    Generate predictions for the incoming request using the model.
    """
    data = pd.DataFrame(body)
    predictions = model.predict(data).tolist()
    return {"predictions": predictions}