# Plan d'Intégration BLE (Bluetooth Low Energy) - GyroBot

## 1. Objectif
Mettre en place une interface de communication robuste et performante via Bluetooth Low Energy pour le pilotage du robot et la surveillance via télémétrie, en complément ou remplacement de l'interface Web.

## 2. Architecture Technique
*   **Pattern Singleton** : Création d'une classe `BleManager` unique. Cela garantit un point d'accès centralisé et évite les initialisations multiples de la pile BLE, très coûteuses en RAM.
*   **Serveur GATT** : 
    *   **Service Unique** : Regroupe les fonctionnalités de contrôle et de données.
    *   **Caractéristique TX (Notify)** : Utilisée pour pousser la télémétrie vers le client connecté au format CSV.
    *   **Caractéristique RX (Write)** : Utilisée pour recevoir les commandes du client BLE (ex: `START_SEQ1`, `STOP`, `CALIBRATE`, etc.) — identiques aux commandes WiFi existantes.

## 3. Stratégie de Données (Haute Efficacité)
*   **Standardisation CSV** : Utilisation exclusive de `Telemetry::getCSV()`. Le format CSV est 40% à 50% plus léger que le JSON, ce qui réduit le temps d'occupation radio et la fragmentation de la mémoire Heap.
*   **Négociation MTU** : Configuration d'un MTU (Maximum Transmission Unit) de 256 octets. Le MTU par défaut de 23 octets est insuffisant pour une ligne CSV complète et provoquerait des troncatures.
*   **Throttling** : Les notifications seront limitées à 50Hz (tous les 20ms) pour correspondre au cycle de la télémétrie et ne pas saturer la bande passante BLE.

## 4. Sécurité et Robustesse
*   **Thread Safety** : La tâche BLE s'exécutant sur un thread séparé (souvent sur le Core 0), l'accès à la télémétrie sera protégé par le **Mutex** déjà présent dans la classe `Telemetry`.
*   **Gestion des Callbacks** : Les fonctions de réception (onWrite) ne contiendront aucune logique lourde. Elles se contenteront de lever des drapeaux ou de remplir une file d'attente pour que la boucle principale traite la commande.
*   **Auto-Advertising** : Relance systématique de la visibilité du robot dès qu'un client se déconnecte. Un délai de 500ms est appliqué après la déconnexion pour éviter les boucles de reconnexion rapides. La réactivité est maintenue tout en prévenant l'instabilité.
*   **Flag WiFi/BLE** : Une variable de configuration `ENABLE_WIFI` contrôle si le WebServer est initialisé. Cela évite les conflits de ressources radio et simplifie le débogage. En compétition, un seul mode peut être activé selon les besoins (WiFi pour développement, BLE pour autonomie en batterie).
*   **Arrêt d'Urgence (Emergency Stop)** : Les commandes BLE ont une **priorité haute**, particulièrement le `STOP`. Le robot revient immédiatement à une position d'attente sécurisée sans attendre la fin de la séquence en cours.

## 5. Étapes de Développement
1.  **Header** : Définir `BleManager.h` avec suppression des constructeurs de copie (Sécurité Singleton).
2.  **Initialisation** : Intégrer `BleManager::getInstance().begin()` dans le `setup()` après l'init des capteurs, conditionnée par `ENABLE_WIFI` (si WiFi est désactivé, BLE s'initialise).
3.  **Telemetry TX** : Dans la boucle `loop()`, envoyer la ligne CSV via notification BLE si un client est connecté.
4.  **Command RX (Queue-based)** : 
    *   `onWrite()` ajoute **uniquement** la commande à une queue (`std::queue<String>`), puis retour immédiatement.
    *   Exception : la commande `STOP` déclenche **immédiatement** le drapeau d'arrêt d'urgence (sans passer par la queue).
    *   Dans `loop()`, traiter les commandes depuis la queue à chaque itération.
5.  **Parsing Unifié** : Créer une fonction `processBleCommand()` dans `main.cpp` qui traite les mêmes commandes que le WebServer (ex: "S1", "S2", "S3", "FLECHE", etc.).
6.  **Redesign Exécution** : Les classes `Sequence*` doivent **vérifier régulièrement** un drapeau d'arrêt d'urgence (`EmergencyStop::getInstance().isTriggered()`) dans leurs boucles internes et leurs `wait()`. À la détection du drapeau, elles doivent :
    *   Arrêter immédiatement les moteurs (`_robot.stop()`).
    *   Retourner sans erreur (état d'attente sécurisé = moteurs coupés).
7.  **Robot::wait() Interruptible** : Refactoriser la méthode `wait(ms)` pour qu'elle vérifie le drapeau d'arrêt d'urgence toutes les 10-50ms au lieu d'attendre le délai complet.
8.  **Flag Reset** : Le drapeau d'arrêt est réinitialisé **avant** le lancement d'une nouvelle séquence, permettant un relancement normal.

## 6. Défense du Projet (Questions/Réponses pour l'oral)
*   **Q : Pourquoi avoir abandonné le JSON pour le BLE ?**
    *   **R** : "Pour l'efficacité. Le JSON impose une surcharge de métadonnées (clés) inutile en BLE. Le CSV permet de transmettre plus de données utiles dans un seul paquet MTU."
*   **Q : Comment gérez-vous les risques de crash liés à la mémoire (Heap) ?**
    *   **R** : "En évitant les allocations dynamiques répétées dans les callbacks et en utilisant le Singleton pour stabiliser l'empreinte mémoire de la pile BLE."
*   **Q : Le WiFi et le BLE peuvent-ils fonctionner ensemble ?**
    *   **R** : "Oui, mais c'est exigeant. Nous utilisons des verrous (Mutex) pour que les deux interfaces accèdent à la même source de vérité (la télémétrie) sans corruption."

## 6. Redesign Critique : Gestion de l'Arrêt d'Urgence
Pour que l'arrêt d'urgence soit **vraiment** efficace, le code existant doit être refactorisé :

*   **Classe Sequence** (base) : Ajouter une méthode protégée `checkEmergencyStop()` appelée dans la boucle principale des `execute()`.
*   **Boucles de Mouvement** : Chaque boucle longue (calibration, rotation continue, etc.) doit appeler `checkEmergencyStop()` régulièrement.
*   **Robot::wait()** : Refactoriser pour faire des mini-boucles avec vérification d'interruption toutes les 10-50ms.
*   **État du Robot** : S'assurer que `_robot.stop()` ramène le robot à un état connu et sécurisé (moteurs coupés, capteurs stables).

## 7. Implémentation : Problème Critique Découvert

**Issue BLE Activation :** La première implémentation tentait d'exécuter les Sequences directement dans le callback `onWrite()`. Cela est **dangereux** car :
*   Le callback s'exécute dans le contexte du task BLE (CPU Core 0), pas dans la boucle principale.
*   L'exécution directe de Sequence bloque la communication BLE et peut causer des timeouts client.
*   La solution : Utiliser un **système de file d'attente simple** où `onWrite()` ajoute la commande à une queue, et la boucle `loop()` la traite.

**Implémentation Corrigée :**
*   BleManager maintient une queue simple (`std::queue<String>` ou `ring buffer`) pour les commandes entrantes.
*   `onWrite()` ajoute la commande à la queue et retour immédiatement.
*   Dans `loop()`, après `BleManager::update()`, vérifier s'il y a une commande à traiter et l'exécuter.
*   Cela garantit que les Sequences s'exécutent dans le contexte de la boucle principale, pas dans le task BLE.

## 8. Points de Vigilance (Anti-Erreurs)
*   Ne pas dépasser 200 octets par ligne CSV pour rester sous la limite MTU négociée.
*   Vérifier que `BLEDevice::init` ne consomme pas trop de RAM par rapport au WebServer.
*   S'assurer que les interruptions encodeurs (`IRAM_ATTR`) restent prioritaires sur le traitement BLE.
*   **L'arrêt d'urgence doit fonctionner à tout moment**, même pendant un `wait()` ou une calibration longue. Tester systématiquement l'arrêt en milieu de séquence.
*   Documenter l'état d'attente après `STOP` : le robot est-il immobile ? En position initiale ? Doit-on réinitialiser avant de relancer une séquence ?