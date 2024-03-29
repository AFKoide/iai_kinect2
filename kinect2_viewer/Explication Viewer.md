Ce programme est conçu pour recevoir des données provenant de deux sujets ROS : un sujet de couleur (image) et un sujet de profondeur. Il synchronise les données d'image couleur et d'image de profondeur avec les informations de la caméra correspondantes, puis les traite et les affiche ou les enregistre.

Voici une explication détaillée de chaque partie du programme :

1. **Bibliothèques incluses** :
   - Les bibliothèques standard (`stdlib.h`, `stdio.h`, etc.) fournissent des fonctionnalités de base en C++.
   - Les bibliothèques `pcl` (Point Cloud Library) sont utilisées pour le traitement des nuages de points.
   - Les bibliothèques `opencv2` sont utilisées pour le traitement d'images.
   - Les bibliothèques ROS (`ros/ros.h`, `sensor_msgs`, `image_transport`, etc.) sont utilisées pour la communication et le traitement des données ROS.

2. **Classe `Receiver`** :
   - Cette classe est responsable de la réception, du traitement et de l'affichage ou de l'enregistrement des données.
   - Elle a trois modes de fonctionnement : IMAGE (seulement les images), CLOUD (seulement les nuages de points) ou BOTH (à la fois les images et les nuages de points).
   - Les membres privés de la classe incluent les sujets ROS pour les images couleur et de profondeur, les indicateurs de mise à jour des données, un mutex pour la synchronisation, etc.
   - Les politiques de synchronisation (`ExactSyncPolicy` et `ApproximateSyncPolicy`) sont utilisées pour synchroniser les messages provenant des sujets ROS.

3. **Fonctions de démarrage et d'arrêt (`start()` et `stop()`)** :
   - La fonction `start()` initialise les abonnements aux sujets ROS et lance la synchronisation des messages.
   - La fonction `stop()` arrête les abonnements et termine les threads en cours.

4. **Fonction de rappel (`callback()`)** :
   - Cette fonction est appelée lorsqu'un message est reçu sur les sujets synchronisés.
   - Elle lit les images et les informations de la caméra et les stocke dans les variables de la classe.

5. **Fonctions d'affichage (`imageViewer()` et `cloudViewer()`)** :
   - Ces fonctions affichent les images et les nuages de points à l'écran.
   - Elles utilisent OpenCV et PCL pour l'affichage graphique.

6. **Fonctions utilitaires (`readImage()`, `readCameraInfo()`, `dispDepth()`, `combine()`, `createCloud()`, `saveCloudAndImages()`, `createLookup()`)** :
   - Ces fonctions sont utilisées pour lire les images et les informations de la caméra, afficher les images de profondeur, générer les nuages de points, sauvegarder les données, etc.

7. **Fonction `main()`** :
   - La fonction `main()` initialise le nœud ROS, analyse les arguments de ligne de commande pour déterminer les paramètres du programme, puis crée une instance de la classe `Receiver` avec les paramètres appropriés et lance la réception des données.

En résumé, ce programme permet de recevoir, synchroniser et traiter des données d'image couleur et de profondeur provenant de capteurs Kinect2 via des sujets ROS, et offre des fonctionnalités d'affichage et d'enregistrement des données.

---

1. **Fonction `start()`:**
   - Démarre la réception des images et des informations de caméra.
   - Crée les abonnements aux topics des images couleur, des images de profondeur et des informations de caméra.
   - Configure les politiques de synchronisation exacte ou approximative selon la valeur de `useExact`.
   - Lance un fil d'exécution asynchrone pour gérer la boucle de réception des images.
   - Démarre le mode d'affichage spécifié (`cloudViewer()`, `imageViewer()` ou les deux).

2. **Fonction `stop()`:**
   - Arrête proprement la réception des images.
   - Arrête le fil d'exécution asynchrone.
   - Supprime les abonnements et les synchroniseurs.
   - Attend la fin du thread d'affichage si le mode était `BOTH`.

3. **Fonction `callback()`:**
   - Appelée lors de la réception d'un nouveau message sur les topics des images couleur, des images de profondeur et des informations de caméra.
   - Extrait les données des messages et les stocke dans les variables `color` et `depth`.

4. **Fonction `imageViewer()`:**
   - Affiche les images couleur et de profondeur en temps réel dans une fenêtre OpenCV.
   - Récupère les images stockées dans `color` et `depth`, les combine et les affiche avec le nombre d'images par seconde (FPS).
   - Gère les événements clavier pour permettre à l'utilisateur de quitter l'application ou de sauvegarder les données.

Ces fonctions travaillent ensemble pour démarrer la réception des images, afficher les images en temps réel et arrêter proprement le processus lorsque nécessaire.

---

1. **Fonction `cloudViewer()`:**
   - Affiche le nuage de points en temps réel à l'aide de la bibliothèque de visualisation PCL.
   - Crée un visualiseur PCL, ajoute le nuage de points, et gère les événements clavier.
   - Récupère les données des images couleur et de profondeur, crée le nuage de points correspondant et met à jour le visualiseur.

2. **Fonction `keyboardEvent()`:**
   - Gère les événements clavier lors de l'affichage du nuage de points.
   - Détecte les pressions de touches spécifiques comme "q" pour quitter ou "s" pour sauvegarder les données.

3. **Fonctions `readImage()` et `readCameraInfo()`:**
   - Convertissent les messages ROS des images et des informations de caméra en objets OpenCV (`cv::Mat`).
   - Utilisent la bibliothèque `cv_bridge` pour faciliter la conversion des formats d'image et d'informations de caméra.

4. **Fonction `dispDepth()`:**
   - Affiche une image de profondeur en fausses couleurs.
   - Normalise les valeurs de profondeur et applique une carte de couleur (jet) pour visualiser la profondeur de manière intuitive.

5. **Fonction `combine()`:**
   - Combine une image couleur et une image de profondeur en une seule image affichable.
   - Moyenne les valeurs de pixel des deux images pour créer une image combinée.

6. **Fonction `createCloud()`:**
   - Crée un nuage de points à partir d'une image de profondeur et d'une image couleur.
   - Parcourt chaque pixel de l'image de profondeur, calcule les coordonnées 3D correspondantes à partir de la profondeur et des informations de caméra, et attribue la couleur du pixel correspondant dans l'image couleur au point 3D du nuage de points.

---

1. **Fonction `cloudViewer()`:**
   - Affiche le nuage de points en temps réel à l'aide de la bibliothèque de visualisation PCL.
   - Crée un visualiseur PCL, ajoute le nuage de points, et gère les événements clavier.
   - Récupère les données des images couleur et de profondeur, crée le nuage de points correspondant et met à jour le visualiseur.

2. **Fonction `keyboardEvent()`:**
   - Gère les événements clavier lors de l'affichage du nuage de points.
   - Détecte les pressions de touches spécifiques comme "q" pour quitter ou "s" pour sauvegarder les données.

3. **Fonctions `readImage()` et `readCameraInfo()`:**
   - Convertissent les messages ROS des images et des informations de caméra en objets OpenCV (`cv::Mat`).
   - Utilisent la bibliothèque `cv_bridge` pour faciliter la conversion des formats d'image et d'informations de caméra.

4. **Fonction `dispDepth()`:**
   - Affiche une image de profondeur en fausses couleurs.
   - Normalise les valeurs de profondeur et applique une carte de couleur (jet) pour visualiser la profondeur de manière intuitive.

5. **Fonction `combine()`:**
   - Combine une image couleur et une image de profondeur en une seule image affichable.
   - Moyenne les valeurs de pixel des deux images pour créer une image combinée.

6. **Fonction `createCloud()`:**
   - Crée un nuage de points à partir d'une image de profondeur et d'une image couleur.
   - Parcourt chaque pixel de l'image de profondeur, calcule les coordonnées 3D correspondantes à partir de la profondeur et des informations de caméra, et attribue la couleur du pixel correspondant dans l'image couleur au point 3D du nuage de points.