import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

class LinearRegression:
    def __init__(self,lr=0.001,n_iter=1000):
        self.weights = None
        self.bias = None
        self.lr = lr
        self.n_iter = n_iter

    def _to_2d(self, X):
        if isinstance(X, pd.Series):
            X = X.to_numpy()
        X = np.asarray(X)
        if X.ndim == 1:
            X = X.reshape(-1, 1)
        return X

    def fit(self,X,y):
        X = self._to_2d(X)  # shape -> (n_samples, n_features)
        y = np.asarray(y).reshape(-1)  # shape -> (n_samples,)
        n_samples, n_features = X.shape
        self.weights = np.zeros(n_features)
        self.bias = 0

        for i in range(self.n_iter):
            y_pred = np.dot(X, self.weights) + self.bias
            dw = (1 / n_samples) * (np.dot(X.T, y_pred - y))
            db=(1 / n_samples) * (np.sum(y_pred - y))
            self.weights -= self.lr * dw
            self.bias -= self.lr * db


    def predict(self,X):
        X = self._to_2d(X)
        y_pred = np.dot(X, self.weights) + self.bias
        return y_pred





