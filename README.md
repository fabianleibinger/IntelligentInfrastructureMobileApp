# IntelligentInfrastructureMobileApp

This is a project developed from a group of students from Karlsruher Institut für Technologie (KIT) together with Forschungszentrum Informatik (FZI). The project lasted from November 2020 until March 2021. The aim was to create a mobile application for an intelligent infrastructure in terms of a parking garage vehicles and drivers can communicate with.

The created system can be divided in a frontend in terms of an app, which can be run on Android and IOS devices, and a backend server, which is accessed by the app via HTTP requests. Furthermore, the backend can be connected to a parking management system using ROS.

## Frontend: Flutter app
Users of the intelligent infrastructure can use the services with an installable mobile application. The app was developed using Flutter and is a so-called cross-platform application meaning its source can be converted to both IOS and Android system´s code. Using this technology, we could combine the advantages of a native app being better performance and use of system´s internal services (e.g. push notifications) as well as the wide range of devices the app can be deployed on.

## Backend: Python Flask server using ROS for communication
