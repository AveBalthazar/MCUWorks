from sklearn.svm import SVC
from sklearn.neighbors import KNeighborsClassifier
from sklearn.linear_model import LogisticRegression

# SVC — большее разнообразие
svc_kernels = ['linear', 'rbf', 'poly']
svc_params = [
    SVC(kernel='linear', C=0.1),
    SVC(kernel='linear', C=10),
    SVC(kernel='rbf', C=1, gamma=0.01),
    SVC(kernel='rbf', C=10, gamma=0.1),
    SVC(kernel='poly', degree=3, C=1, gamma=0.01)
]

# KNN — разные метрики и числа соседей
knn_params = [
    KNeighborsClassifier(n_neighbors=3, metric='euclidean'),
    KNeighborsClassifier(n_neighbors=7, metric='manhattan'),
    KNeighborsClassifier(n_neighbors=15, metric='chebyshev'),
    KNeighborsClassifier(n_neighbors=25, metric='minkowski')
]

# Logistic Regression — разные регуляризации
logreg_params = [
    LogisticRegression(C=0.01, solver='liblinear', penalty='l2'),
    LogisticRegression(C=1, solver='liblinear', penalty='l2'),
    LogisticRegression(C=100, solver='lbfgs', penalty='l2', max_iter=1000),
    LogisticRegression(penalty=None, solver='lbfgs', max_iter=1000)
]

models = svc_params + knn_params + logreg_params

from sklearn.model_selection import cross_val_score
from sklearn.preprocessing import StandardScaler
import pandas as pd
import numpy as np

scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

results = []
for model in models:
    scores = cross_val_score(model, X_scaled, y, cv=5, scoring='f1_macro')
    results.append({
        'Model': type(model).__name__,
        'Params': str(model.get_params()),
        'Mean F1': np.mean(scores),
        'Std F1': np.std(scores)
    })

df_results = pd.DataFrame(results).sort_values('Mean F1', ascending=False)
print(df_results[['Model', 'Mean F1', 'Std F1']])

import matplotlib.pyplot as plt
import seaborn as sns

plt.figure(figsize=(10,6))
sns.barplot(data=df_results, x='Mean F1', y='Model', orient='h', ci=None)
plt.title('Сравнение моделей по F1 (5-fold CV)')
plt.xlabel('F1 (macro)')
plt.ylabel('')
plt.grid(True, axis='x', linestyle='--', alpha=0.6)
plt.show()

from sklearn.metrics import ConfusionMatrixDisplay
best_model = models[np.argmax(df_results['Mean F1'])]
best_model.fit(X_scaled, y)
ConfusionMatrixDisplay.from_estimator(best_model, X_scaled, y, cmap='Blues')
plt.title(f'Confusion Matrix: {type(best_model).__name__}')
plt.show()
