
import numpy as np
from sklearn.neural_network import MLPClassifier
from sklearn import svm
from sklearn.ensemble import RandomForestClassifier
from sklearn.ensemble import ExtraTreesClassifier
from sklearn.neighbors import KNeighborsClassifier
from sklearn.metrics import accuracy_score
import matplotlib.pyplot as plt
from sklearn import datasets, svm, metrics
import pandas as pd
from ast import literal_eval


# data_training = pd.read_csv("training_g1.csv")
# data_training = pd.read_csv("group2_training_R.csv")
data_training = pd.read_csv("training_group3_tape.csv")
# Preview the first 5 lines of the loaded data
# print (data.head())

# data_testing = pd.read_csv("testing_g1.csv")
# data_testing = pd.read_csv("group2_testing_R.csv")
data_testing = pd.read_csv("testing_group3_T.csv")

X_train = []
data1 = data_training.data

# l = np.size(x)
for i in data1:
    # print (i)
    X_train.append(literal_eval(i))

y_train = data_training.Label


X_test = []
data2 = data_testing.data


# l = np.size(x)
for j in data2:
    # print (j)
    X_test.append(literal_eval(j))

y_test = data_testing.Label

# print (x)

# classifiers:

# C = 5  # SVM regularization parameter
# clf = svm.SVC(kernel='poly', degree=3, gamma='auto', C=C)

clf = MLPClassifier(solver='lbfgs',alpha=1e-5, max_iter=300, hidden_layer_sizes=(20,10), random_state=1)

# clf = RandomForestClassifier(max_depth=10, n_estimators=200)

# clf = ExtraTreesClassifier(n_estimators=200, random_state=0)
# clf = KNeighborsClassifier(n_neighbors=5, leaf_size=20)

clf.fit(X_train, y_train)

predicted = clf.predict(X_test)
# print (clf.predict_proba([[2.8, 89]]))

print(accuracy_score(y_test, predicted))

print("Classification report for classifier %s:\n%s\n"
      % (clf, metrics.classification_report(y_test, predicted)))

# disp.figure_.suptitle("Confusion Matrix")
# print("Confusion matrix:\n%s" % disp.confusion_matrix)

titles_options = [("Confusion matrix, without normalization", None),
                  ("Normalized confusion matrix", 'true')]
for title, normalize in titles_options:
    disp = metrics.plot_confusion_matrix(clf, X_test, y_test,
                                 display_labels=('52mmm', '56mm', '61mm'),
                                 cmap=plt.cm.Blues,
                                 normalize=normalize)
    disp.ax_.set_title(title)

    print(title)
    print(disp.confusion_matrix)

plt.show()