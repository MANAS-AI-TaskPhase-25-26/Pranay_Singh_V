import numpy as np


class LinearRegression:
    def __init__(self,lr=0.01,n=100000):
        self.weights=None
        self.bias=None
        self.lr=lr
        self.n=n


#For finding the optimal weights and bias using batch gradient descent
    def fit(self,X,y):
        X = (X - np.mean(X, axis=0)) / np.std(X, axis=0)
        y = (y - np.mean(y)) / np.std(y)
        n_samples,n_features=X.shape
        self.weights=np.zeros(n_features)
        self.bias=0

        for i in range(self.n):
            y_pred=np.dot(X,self.weights) + self.bias
            dw=(1/n_samples)*(np.dot(X.T,y_pred-y))
            db=(1/n_samples)*np.sum(y_pred-y)
            self.weights=self.weights-self.lr*dw
            self.bias = self.bias - self.lr*db

    #For final predictions
    def predict(self,X):
        predictions = np.dot(X, self.weights) + self.bias
        return predictions