#!/usr/bin/env python3
"""
Train membership inference classifier from shadow-model outputs.
Assumes softmax vectors saved as numpy arrays for in/out sets.
"""
import argparse
import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import roc_auc_score, accuracy_score
import joblib

def load_vectors(path):
    return np.load(path)  # shape (N, num_classes)

def prepare_features(in_path, out_path):
    X_in = load_vectors(in_path)
    X_out = load_vectors(out_path)
    X = np.vstack([X_in, X_out])
    y = np.concatenate([np.ones(len(X_in)), np.zeros(len(X_out))])
    return X, y

def main(args):
    X, y = prepare_features(args.in_vectors, args.out_vectors)
    model = LogisticRegression(max_iter=1000, solver='lbfgs', C=1.0)
    model.fit(X, y)
    preds = model.predict_proba(X)[:,1]
    auc = roc_auc_score(y, preds)
    acc = accuracy_score(y, preds.round())
    joblib.dump(model, args.output_model)
    print(f"AUC: {auc:.4f}, Acc: {acc:.4f}")

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--in_vectors', required=True, help='Numpy file with softmax outputs for members.')
    p.add_argument('--out_vectors', required=True, help='Numpy file with softmax outputs for non-members.')
    p.add_argument('--output_model', default='mem_clf.joblib', help='Save path for classifier.')
    main(p.parse_args())