import numpy as np
def sigmoid(z):
    return 1/(1+np.exp(-z))

class LogisticRegression:
    def __init__(self,lr=0.01,n=100000):
        self.weights=None
        self.bias=None
        self.lr=lr
        self.n=n


#For finding the optimal weights and bias using batch gradient descent
    def fit(self,X,y):
        n_samples,n_features=X.shape
        self.weights=np.zeros(n_features)
        self.bias=0

        for i in range(self.n):
            z=np.dot(X,self.weights) + self.bias
            y_pred=sigmoid(z)
            dw=(1/n_samples)*(np.dot(X.T,y_pred-y))
            db=(1/n_samples)*np.sum(y_pred-y)
            self.weights=self.weights-self.lr*dw
            self.bias = self.bias - self.lr*db

    #For predicting the final classifications
    def predict(self,X):
        z = np.dot(X, self.weights) + self.bias
        predictions= sigmoid(z)
        classification=[0 if i<=0.5 else 1 for i in predictions]
        return classification