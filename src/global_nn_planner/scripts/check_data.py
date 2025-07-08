import joblib

path = "/home/gr-agv-x9xy/Downloads/pretraining_dataset/db_cnn_data/data_64.pkl"
data = joblib.load(open(path, 'rb'))

if isinstance(data, dict):
    print("Keys:", data.keys())
elif isinstance(data, (tuple, list)):
    print("Type:", type(data), "Length:", len(data))
    print("Sub-elements types:", [type(d) for d in data])
