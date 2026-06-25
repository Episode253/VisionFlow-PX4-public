import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import json

def normalize(dataset):
    data_normalized = ((dataset-dataset.min())/(dataset.max()-dataset.min()))
    return data_normalized
# 数据加载与预处理
def load_data():
    data = pd.read_csv('all_only_learning.csv')
    #data = pd.read_csv('all_data.csv')
    X = data.iloc[:, :-5].values  # 假设最后 6 列是标签
    y = data.iloc[:, -5:].values

    # 划分训练集和测试集
    # X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
    # 划分训练集和测试集，不打乱数据顺序
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42, shuffle=False)


    # 特征和标签归一化
    # z=（x-u）/s
    scalers_X = [StandardScaler() for _ in range(X_train.shape[1])]
    scalers_y = [StandardScaler() for _ in range(y_train.shape[1])]
   
    y_mean=np.zeros(5)
    y_std=np.zeros(5)
    x_mean=np.zeros(30)
    x_std=np.zeros(30)
    for i in range(X_train.shape[1]):
        X_train[:, i] = scalers_X[i].fit_transform(X_train[:, i].reshape(-1, 1)).flatten()
        X_test[:, i] = scalers_X[i].transform(X_test[:, i].reshape(-1, 1)).flatten()
        x_mean[i] = scalers_X[i].mean_
        x_std[i] = scalers_X[i].scale_

    for i in range(y_train.shape[1]):
        y_train[:, i] = scalers_y[i].fit_transform(y_train[:, i].reshape(-1, 1)).flatten()
        y_test[:, i] = scalers_y[i].transform(y_test[:, i].reshape(-1, 1)).flatten()
        y_mean[i] = scalers_y[i].mean_
        y_std[i] = scalers_y[i].scale_

    return X_train, X_test, y_train, y_test, scalers_X, scalers_y,x_mean,x_std, y_mean,y_std



# 网络结构定义
class MLP(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(MLP, self).__init__()
        self.layers = nn.Sequential(
            nn.Linear(input_dim, 32),
            nn.ELU(),
            nn.Linear(32, 32),
            nn.ELU(),
            nn.Linear(32, output_dim)
        )
    
    def forward(self, x):
        return self.layers(x)

# 训练过程
def train_model(model, criterion, optimizer, train_loader, epochs=100):
    model.train()
    for epoch in range(epochs):
        for data, target in train_loader:
            optimizer.zero_grad()
            output = model(data)
            loss = criterion(output, target)
            loss.backward()
            optimizer.step()
        print(f'Epoch {epoch+1}, Loss: {loss.item()}')

# 主函数
def main():
    X_train, X_test, y_train, y_test, scalers_X, scalers_y, x_mean, x_std, y_mean, y_std = load_data()
    # print(x_mean)
    # print(x_std)
    # print(y_mean)
    # print(y_std)



    data = {
    "x_mean": x_mean.tolist(),
    "x_std": x_std.tolist(),
    "y_mean": y_mean.tolist(),
    "y_std": y_std.tolist()}
    json_data = json.dumps(data, indent=4)

    # 将 JSON 字符串写入到 txt 文件中
    with open("NN_normalization.txt", "w") as file:
        file.write(json_data)
    print("数组已保存为 JSON 格式到 arrays.txt 文件中。")
    

   
    # 创建数据加载器
    train_dataset = TensorDataset(torch.tensor(X_train, dtype=torch.float32), torch.tensor(y_train, dtype=torch.float32))
    train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
    
    # 模型初始化
    model = MLP(input_dim=30, output_dim=5)
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    
    # 训练模型
    train_model(model, criterion, optimizer, train_loader)
    
    # 测试模型
    model.eval()
    with torch.no_grad():
        predictions = model(torch.tensor(X_test, dtype=torch.float32))
        predictions = np.column_stack([
            scalers_y[i].inverse_transform(predictions[:, i].reshape(-1, 1)).flatten()
            for i in range(predictions.shape[1])
        ])
        y_test_original = np.column_stack([
            scalers_y[i].inverse_transform(y_test[:, i].reshape(-1, 1)).flatten()
            for i in range(y_test.shape[1])
        ])


    for name, param in model.named_parameters():
        print(name)  # to check the parameter naming
    parameters = {name: param.detach().numpy().tolist() for name, param in model.named_parameters()}
    # Save the parameters dictionary to a text file
    with open('parameters.txt', 'w') as f:
        json.dump(parameters, f)   
    
    parameters = {name: param.detach().numpy() for name, param in model.named_parameters()}
    parameters = {k: v.T if 'weight' in k else v for k, v in parameters.items()} 

    #print(parameters)
    # 可视化结果
    m=np.array([0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0])
    m=torch.from_numpy(m).float() 
    a=model(m)
    a_original=[]
    a_original2=np.zeros(5)
    
    '''
    归一化与逆归一化
    print(y_mean)
    print(y_std)
    z=(x-u)/s
    x = z*s + u
    '''
    '''
    写一个函数,将均值以及标准差保存到txt文件中,其中MPC直接从python读取参数,我们在c++中通过读取文件获取参数计算网络参数，用于更新
    
    '''
    for i in range(5):
        output_detached = a[i].detach().numpy()  # Detach and convert to numpy
        transformed = scalers_y[i].inverse_transform(output_detached.reshape(-1, 1)).flatten()
        a_original.append(transformed)

        # 逆归一化  x = z*s + u
        a_original2[i] = a[i].detach().numpy()*y_std[i] + y_mean[i]
        #a_original[i] = scalers_y[i].inverse_transform(a[i].reshape(-1, 1)).flatten()
    print(a_original)
    print(a_original2)
    fig, axes = plt.subplots(5, 1, figsize=(10, 15))
    for i in range(5):
        axes[i].plot(y_test_original[:, i], label='Actual')
        axes[i].plot(predictions[:, i], label='Predicted')
        axes[i].set_title(f'Output Dimension {i+1}')
        axes[i].legend()
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
