// This is an example Custom Command Qml file. You have full access to the entire Qml language
// for creating any user interface you like. From the ui you can affect the following changes
// with respect to your vehicle:
//    1) Sending COMMAND_LONG commands out over mavlink using QGCButton control
//    2) Modifying parameters
//
// When developing custom Qml file implementations. You must restart QGroundControl to pick up
// the changes. You need to do this even if you select Clear Qml file. Not sure what at the this
// point. Qt must be caching the files somewhere.

import QtQuick 2.2

//debug 等待50秒自动关闭23电机
/* static int timer = 0;
if(timer++ > 400*50){
    if(i == 2 || i == 3){
        outputs[i] = -1;
    }
} */

import QGroundControl.Controls 1.0
import QGroundControl.FactSystem 1.0
import QGroundControl.FactControls 1.0
import QGroundControl.Controllers 1.0

FactPanel {
    id: panel
    
    property var qgcView: null // Temporary hack for broken QGC parameter validation implementation

    CustomCommandWidgetController { id: controller; factPanel: panel }

    // Your own custom changes start here - everything else above is always required

    Column {
        // The QGCButton control is provided by QGroundControl.Controls. It is a wrapper around
        // the standard Qml Button element which using the default QGC font and color palette.
        

        // The FactTextField control is provides by GroundControl.FactControls. It is a wrapper
        // around the Qml TextField element which allows you to bind it directly to any parameter.
        // The parameter is changed automatically when you click enter or click away from the field.
        // Understand that there is currently no value validation. So you may crash your vehicle by
        // setting a parameter to an incorrect value. Validation will come in the future.

        // Be very careful when referencing parameters. If you specify a parameter which does not exist
        // QGroundControl will warn and shutdown.

        /* FactTextField {
            // The -1 signals default component id.
            // You can replace it with a specific component id if you like
            fact: controller.getParameterFact(-1, "MAV_SYS_ID")
        } */

        QGCButton {
            text: "UAV_START "
            onClicked: controller.sendCommand(600, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        }

        QGCButton {
            text: "Master Start"
            onClicked: controller.sendCommand(600, 0, 0, 6, 0, 0, 0, 0, 0, 0)
        }
        
        QGCButton {
            text: "Slave Start"
            onClicked: controller.sendCommand(600, 0, 0, 7, 0, 0, 0, 0, 0, 0)
        }

        QGCButton {
            text: "All_Stop "
            onClicked: controller.sendCommand(600, 0, 0, 1, 0, 0, 0, 0, 0, 0)
        }

        QGCButton {
            text: "Slave Back X"
            onClicked: controller.sendCommand(600, 0, 0, 2, 0, 0, 0, 0, 0, 0)
        }
        
        QGCButton {
            text: "Slave Forward X"
            onClicked: controller.sendCommand(600, 0, 0, 3, 0, 0, 0, 0, 0, 0)
        }

        QGCButton {
            text: "Slave Back Y"
            onClicked: controller.sendCommand(600, 0, 0, 4, 0, 0, 0, 0, 0, 0)
        }
        
        QGCButton {
            text: "Slave Forward Y"
            onClicked: controller.sendCommand(600, 0, 0, 5, 0, 0, 0, 0, 0, 0)
        }

        QGCButton {
            text: "ALLRETURN"
            onClicked: controller.sendCommand(600, 0, 0, 8, 0, 0, 0, 0, 0, 0)
        }

        QGCButton {
            text: "ALLLAND"
            onClicked: controller.sendCommand(600, 0, 0, 9, 0, 0, 0, 0, 0, 0)
        }

        QGCButton {
            text: "FOLLOW_USV"
            onClicked: controller.sendCommand(600, 0, 0, 10, 0, 0, 0, 0, 0, 0)
        }
    }

    // Your own custom changes end here - everything else below is always required
}
