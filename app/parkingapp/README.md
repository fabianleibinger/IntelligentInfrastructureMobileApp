# Parking App

## Usage, modification and further development

The Parking App is responsible for managing the users personal vehicles and therefore enables creating, editing and deleting vehicles in a local database, which is accessible via the DataHelper class.

The vehicles can be divided into two sub-categories: standard vehicles and chargeable vehicles, which differ by the amount of extra features, e.g. for charging. New sub-categories could easily be integrated into the app by deriving from the Vehicle class, which is abstract.

The main goal of the Parking App is to send park requests for the users vehicles to the parking garage. This is achieved by the ApiProvider class that handles communication with the backend via HTTP get and post requests. 

To keep the vehicles state consistent during park operations, the ParkManager class provides the necessary logic.

During a park operation the user can track the current and target position of his vehicle on a map of the parking garage, implemented on the ParkPages.

Push notifications inform about important operations and allow the user to park out a vehicle from home screen with only one click. 