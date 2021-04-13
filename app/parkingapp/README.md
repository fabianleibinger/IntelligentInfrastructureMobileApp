# Parking App

## Usage, modification and further development

### Vehicle management
The Parking App is responsible for managing the users personal vehicles and therefore enables creating, editing and deleting vehicles (VehiclePage and EditVehicle widgets) in a local database (DatabaseProvider class), which is accessible via the DataHelper class.

Users can share their vehicles with other users by scanning a QR Code, implemented in the QRPage widget, the QRGenerator and QRScanner class.

The vehicles (Vehicle class) are divided into two categories: standard vehicles and chargeable vehicles (StandardVehicle and ChargeableVehicle class), differing by the amount of extra features, e.g. for charging. New categories could easily be integrated into the app by deriving from the Vehicle class, which is abstract. Keep in mind that new vehicle properties have to be included in the DatabaseProvider and the responsible dialogs (Dialogs package).

The user can select one of his vehicles in the AppDrawer widget. Thereafter the vehicles' properties are summarized on the MainPage widget, which provides the park in button and the compatible parking garage information in addition, e.g. available parking spots.

### Backend communication
The main goal of the Parking App is to send park requests for the users vehicles to the parking garage. All requests are handled by the ApiProvider class that communicates with the backend via HTTP. The url of the backend can be changed with the _serverUrl variable (ApiProvider) as well as the port (_serverPort).

### Park operations
To keep the vehicles state consistent during park operations, the ParkManager class provides the necessary logic.

During a park process the user can track the current and target position of his vehicle on a map of the parking garage, implemented on the ParkPages (ParkInPage and ParkOutPage classes) and in the ParkManager.

Push notifications (Notifications class) inform about important operations and allow the user to park out a vehicle from home screen with only one click. Notifications can be enabled or disabled on the SettingsPage widget. 