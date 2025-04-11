# Creación de nuevos controladores

Este apartado describe cómo escribir un nuevo controlador personalizado que toma señales de entrada y genera las señales de control enviadas a los actuadores utilizando ros2_control.

## Introduccion

Los controladores son el núcleo del sistema ros2_control. El paquete ros2_controllers incluye un conjunto de controladores estándar comunes y completos que satisfacen muchos casos de uso habituales. Algunos ejemplos de estos controladores son el controlador de esfuerzo, el controlador de trayectoria y el controlador de accionamiento diferencial, y la lista de controladores proporcionados crece con cada versión de ROS2. Sin embargo, si bien algunos controladores se pueden aplicar ampliamente a muchos tipos diferentes de aplicaciones robóticas, también es cierto que algunas aplicaciones también requieren controladores específicos, según la naturaleza y los requisitos de una aplicación.

Afortunadamente, ros2_control es lo suficientemente flexible como para permitir crear controladores personalizados para ampliar su funcionalidad. De esta manera, también se puede usar ros2_control para resolver problemas de control más complejos sin demasiados problemas. En este apartado, se describirá cómo crear un controlador personalizado. 

## Controlador propio en 5 pasos

Para implementar un nuevo controlador se deberán seguir los siguientes pasos:

* Crear un paquete para el controlador personalizado.
* Escribir un archivo de encabezado .hpp.
* Definir un archivo fuente el controlador .cpp.
* Preparar los archivos CMakeLists.txt y package.xml para la compilación.
* Registrar el controlador como un plugin.

## Crear un paquete para el controlador personalizado

Como es habitual, el enfoque recomendado es configurar un nuevo paquete para mantener nuestro nuevo controlador modular e intercambiable.

Primero, ir al directorio src dentro de ros2_ws:

```
cd ~/ros2_ws/src
```

Ahora crea el nuevo paquete. Este nuevo paquete debe tener ament_cmake como tipo de compilación y, como práctica recomendada, se sugiere terminar el nombre del nuevo paquete con "_controller". También se necesitarán unas cuantas dependencias:

* control_msgs
* controller_interface
* hardware_interface
* pluginlib
* rclcpp
* rclcpp_lifecycle
* realtime_tools
* example_interfaces

Ejecutar el siguiente comando:

```
ros2 pkg create --build-type=ament_cmake rrbot_controller --dependencies control_msgs controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools example_interfaces
```
El directorio del nuevo paquete ahora debería tener un archivo CMakeLists.txt y un archivo package.xml y un directorio src y un directorio include/rrbot_controller.

## Escribir un archivo de encabezado .hpp

Ahora, dentro de la carpeta include/rrbot_controller, crear un archivo llamado rrbot_controller.hpp.

```
touch ~/ros2_ws/src/rrbot_controller/include/rrbot_controller/rrbot_controller.hpp
```

Luego añadir el siguiente fragmento de código en ese archivo:

```
// Copyright (c) 2021, Bence Magyar and Denis Stogl
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RRBOT_CONTROLLER__RRBOT_CONTROLLER_HPP_
#define RRBOT_CONTROLLER__RRBOT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

namespace rrbot_controller {
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RRBotController : public controller_interface::ControllerInterface {
public:
  RRBotController();

  CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:
  std::vector<std::string> joint_names_;
  std::string interface_name_;

  // Command subscribers and Controller State publisher
  using ControllerCommandMsg = control_msgs::msg::JointJog;

  rclcpp::Subscription<ControllerCommandMsg>::SharedPtr command_subscriber_ =
      nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandMsg>>
      input_command_;

  using ControllerStateMsg = control_msgs::msg::JointControllerState;
  using ControllerStatePublisher =
      realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;
};

} // namespace rrbot_controller

#endif // RRBOT_CONTROLLER__RRBOT_CONTROLLER_HPP_
```
### Explicación del código

A continuación se va a describir este código por trozos.
```
#ifndef RRBOT_CONTROLLER__RRBOT_CONTROLLER_HPP_
#define RRBOT_CONTROLLER__RRBOT_CONTROLLER_HPP_
```
Este código de encabezado asegura que no se incluya más de una vez.

```
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
// #include "rrbot_controller/visibility_control.h"

#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"
```

Se utilizan las directivas #include para incluir el contenido de los archivos especificados en el compilador.

```
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
```

Esto nos permitirá usar las formas abreviadas CallbackReturn::SUCCESS y CallbackReturn::ERROR en lugar del nombre de espacio de nombres completo rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS más adelante en el código.
```
namespace rrbot_controller
```
Esto permite poner nuestra implementación en un espacio de nombres que coincida con el nombre del paquete.
```
class RRBotController : public controller_interface::ControllerInterface
```
Aquí solo estamos declarando la clase RRBotController, que hereda de la clase controller_interface::ControllerInterface. Esta última es la clase de la que todo controlador personalizado debe heredar para poder ser construido correctamente.

```
public:
  RRBotController();

  CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

```

Después, debemos declarar estos métodos públicos, que son los que cualquier controlador personalizado debe implementar o sobrescribir en el archivo .cpp.

Explicaremos cada uno de estos métodos en un momento, cuando los implementemos en el código.

```
protected:
  std::vector<std::string> joint_names_;
  std::string interface_name_;

  // Command subscribers and Controller State publisher
  using ControllerCommandMsg = control_msgs::msg::JointJog;

  rclcpp::Subscription<ControllerCommandMsg>::SharedPtr command_subscriber_ =
      nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandMsg>>
      input_command_;

  using ControllerStateMsg = control_msgs::msg::JointControllerState;
  using ControllerStatePublisher =
      realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;
```
Finalmente, declaramos las variables que necesitaremos para este controlador, así como un objeto suscriptor y un objeto publicador.

## Definición del archivo fuente del controlador .cpp

A continuación tenemos que agregar el archivo fuente correspondiente dentro de la carpeta src del paquete y definir todos los métodos que acabamos de declarar en el archivo de encabezado.

Dentro de la carpeta src , crear un archivo llamado rrbot_controller.cpp:

```
touch ~/ros2_ws/src/rrbot_controller/src/rrbot_controller.cpp
```

Luego agregar el siguiente fragmento de código en ese archivo:


```
// Copyright (c) 2021, Bence Magyar and Denis Stogl
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rrbot_controller/rrbot_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace rrbot_controller
{
RRBotController::RRBotController() : controller_interface::ControllerInterface() {}
    
// All the methods added must be inside this namespace
    
}  // namespace rrbot_controller
```

Guardar el archivo. En los siguientes apartados se irán implementando las funciones necesarias dentro de este código.

### Explicación del código

En primer lugar se definen las librerías necesarias y el espacio de nombres que encierra esta clase:

```
#include "rrbot_controller/rrbot_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>
```

Luego agregamos la definición del espacio de nombres que encierra la implementación de los métodos RRBotController de la clase:

```
namespace rrbot_controller
{
RRBotController::RRBotController() : controller_interface::ControllerInterface() {}
    
// All the methods added must be inside this namespace
    
}  // namespace rrbot_controller
```

Tener en cuenta que todos los métodos que agregaremos a este archivo deben ubicarse dentro de este espacio de nombres. La última llave de cierre indica el final del espacio de nombres.

### Método on_init()

Durante la etapa de inicialización, declaramos todos los parámetros que aceptaremos durante la vida útil del controlador. Esto se hace para que el tipo y el nombre del parámetro estén bien definidos en el momento del inicio, lo que reduce las posibilidades de una configuración incorrecta más adelante.

Para continuar con el controlador de la sección anterior, copia y pega el código que se muestra a continuación en el archivo rrbot_controller.cpp. Asegúrate de pegarlo dentro de los corchetes del espacio de nombres.

```
CallbackReturn RRBotController::on_init() {
  try {
    auto_declare("joints", std::vector<std::string>());
    auto_declare("interface_name", std::string());
  } catch (const std::exception &e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n",
            e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}
```

Rodeamos el código con un bloque try/catch para poder notificar al administrador del controlador si ocurre algún problema al procesar el método on_init(). En este ejemplo, on_init() ejecuta la declaración de los parámetros que este controlador requerirá y leerá del archivo de configuración del controlador. Si hay algún error en este segmento de código, este método devolverá un error y la máquina de estados del administrador del controlador cambiará a "finalizado".
Si todo sale bien, devolvemos CallbackReturn::SUCCESS.

### Método on_configure() 

El método on_configure() se utiliza para leer los valores de los parámetros y declarar los suscriptores y publicadores necesarios. Este método se ejecuta antes de la llamada al método update y garantiza que, una vez que el algoritmo de control entre en acción, todo esté configurado y listo para funcionar.

Este es el código C++ que implmenta el template para on_configure(). Añadirlo al código fuente del controlador:
```
CallbackReturn RRBotController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  auto error_if_empty = [&](const auto &parameter, const char *parameter_name) {
    if (parameter.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was empty",
                   parameter_name);
      return true;
    }
    return false;
  };

  auto get_string_array_param_and_error_if_empty =
      [&](std::vector<std::string> &parameter, const char *parameter_name) {
        parameter = get_node()->get_parameter(parameter_name).as_string_array();
        return error_if_empty(parameter, parameter_name);
      };

  auto get_string_param_and_error_if_empty =
  [&](std::string &parameter, const char *parameter_name) {
    parameter = get_node()->get_parameter(parameter_name).as_string();
    return error_if_empty(parameter, parameter_name);
  };

  if (
    get_string_array_param_and_error_if_empty(joint_names_, "joints") ||
      get_string_param_and_error_if_empty(interface_name_, "interface_name")) {
    return CallbackReturn::ERROR;
  }

  // Command Subscriber and callbacks
  auto callback_command =
      [&](const std::shared_ptr<ControllerCommandMsg> msg) -> void {
    if (msg->joint_names.size() == joint_names_.size()) {
      input_command_.writeFromNonRT(msg);
    } else {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Received %zu , but expected %zu joints in command. "
                   "Ignoring message.",
                   msg->joint_names.size(), joint_names_.size());
    }
  };
  command_subscriber_ = get_node()->create_subscription<ControllerCommandMsg>(
      "~/commands", rclcpp::SystemDefaultsQoS(), callback_command);

  // State publisher
  s_publisher_ =
  get_node()->create_publisher<ControllerStateMsg>(
      "~/state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);

  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = joint_names_[0];
  state_publisher_->unlock();

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}
```
#### Explicación del código 
```
  auto error_if_empty = [&](const auto & parameter, const char * parameter_name) {
    if (parameter.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was empty", parameter_name);
      return true;
    }
    return false;
  };
```
En la primera parte del método on_configure(), declaramos una función lambda para verificar si hay parámetros vacíos y devolver un valor booleano.
```
  auto get_string_array_param_and_error_if_empty =
    [&](std::vector<std::string> & parameter, const char * parameter_name) {
      parameter = get_node()->get_parameter(parameter_name).as_string_array();
      return error_if_empty(parameter, parameter_name);
    };
```
A continuación, declaramos una segunda función lambda para recuperar una cadena de parámetros y verificar si el array de parámetros está vacío, utilizando la primera función lambda declarada anteriormente.
```
  auto get_string_param_and_error_if_empty =
    [&](std::string & parameter, const char * parameter_name) {
      parameter = get_node()->get_parameter(parameter_name).as_string();
      return error_if_empty(parameter, parameter_name);
    };
```
Después de eso, declaramos una tercera función lambda para recuperar un parámetro de tipo cadena y realizar la verificación de vacío, la cual también devuelve un valor booleano.
```
  if (
    get_string_array_param_and_error_if_empty(joint_names_, "joints") ||
    get_string_param_and_error_if_empty(interface_name_, "interface_name")) {
    return CallbackReturn::ERROR;
  }
```
En el código anteriorse llama a la segunda y tercera función lambda para evaluar si el array de parámetros "joints" y el parámetro "interface" están vacíos, en cuyo caso el método on_configure() devuelve CallbackReturn::ERROR.
```
  // Command Subscriber and callbacks
  auto callback_command = [&](const std::shared_ptr<ControllerCommandMsg> msg) -> void {
    if (msg->joint_names.size() == joint_names_.size()) {
      input_command_.writeFromNonRT(msg);
    } else {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Received %zu , but expected %zu joints in command. Ignoring message.",
        msg->joint_names.size(), joint_names_.size());
    }
  };
  command_subscriber_ = get_node()->create_subscription<ControllerCommandMsg>(
    "~/commands", rclcpp::SystemDefaultsQoS(), callback_command);
```
En la segunda parte del on_configure() mostrada anteriormente, declaramos una función lambda como la función de devolución de llamada para el comando y declaramos un objeto suscriptor de comandos.
```
  // State publisher
  s_publisher_ =
    get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);

  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = joint_names_[0];
  state_publisher_->unlock();
```
A continuación, declaramos un publicador para transmitir los estados de las articulaciones y lo convertimos en un puntero único. Las últimas tres líneas anteriores se utilizan para evitar que múltiples hilos accedan al mensaje publicado al mismo tiempo, mientras se establece el valor del frame_id en el encabezado.
```
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
```
Al finalizar, el método on_configure() informa al usuario sobre su finalización exitosa y devuelve CallbackReturn::SUCCESS.

### Método command_interface_configuration() 

En este método se define las interfaces de comando necesarias.


Añadir el siguiente bloque de código al archivo rrbot_controller.cpp dentro del espacio de nombres:

```
controller_interface::InterfaceConfiguration RRBotController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(joint_names_.size());
  for (const auto & joint : joint_names_) {
    command_interfaces_config.names.push_back(joint + "/" + interface_name_);
  }

  return command_interfaces_config;
}
```
#### Explicación del código 
```
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
```

Primero se crea un nuevo objeto InterfaceConfiguration y se establece el tipo como INDIVIDUAL. Hay tres opciones para la configuración de la interfaz: ALL, INDIVIDUAL y NONE. ALL y NONE solicitarán acceso a todas las interfaces disponibles o a ninguna de ellas. La configuración INDIVIDUAL necesita una lista detallada de los nombres de las interfaces requeridas. Normalmente, estos se proporcionan como parámetros.
La llamada anterior reservará espacio de memoria para el vector de nombres de interfaces.
```
  command_interfaces_config.names.reserve(joint_names_.size());
  for (const auto & joint : joint_names_) {
    command_interfaces_config.names.push_back(joint + "/" + interface_name_);
  }
```
En el bloque anterior, la primera línea reserva espacio de memoria para el vector de nombres de interfaces.

Luego, a cada articulación se le asigna su propio nombre de interfaz, que se guarda dentro de command_interfaces_config.names. Un nombre de interfaz completo debe tener la estructura <nombre_de_articulación>/<tipo_de_interfaz>.

### Método state_interface_configuration() 

Este método cumple una función similar a la del método anterior, con la diferencia de que este se utiliza para definir qué interfaces de sensores de hardware son requeridas por el controlador.

Añadir el siguiente código a rrbot_controller.cpp:

```
controller_interface::InterfaceConfiguration RRBotController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(joint_names_.size());
  for (const auto & joint : joint_names_) {
    state_interfaces_config.names.push_back(joint + "/" + interface_name_);
  }

  return state_interfaces_config;
}
```
#### Explicación del código 
```
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
```

De nuevo se crea un nuevo objeto InterfaceConfiguration y se establece el tipo como INDIVIDUAL.
```
state_interfaces_config.names.reserve(joint_names_.size());
```
Luego, reservamos memoria para el tamaño de la interfaz.
```
  for (const auto & joint : joint_names_) {
    state_interfaces_config.names.push_back(joint + "/" + interface_name_);
  }
```

Este es un procedimiento estándar para almacenar los nombres de las articulaciones desde archivos de configuración .yaml, así que este método no cambiará mucho en otra implementación personalizada de un controlador.

### Método template get_ordered_interfaces()

A continuación agregar la función template get_ordered_interfaces() que se muestra a continuación al final del método state_interface_configuration() que agregaste anteriormente.
```
template <typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  for (const auto & joint_name : joint_names) {
    for (auto & command_interface : unordered_interfaces) {
      if (
        (command_interface.get_name() == joint_name) &&
        (command_interface.get_interface_name() == interface_type)) {
        ordered_interfaces.push_back(std::ref(command_interface));
      }
    }
  }

  return joint_names.size() == ordered_interfaces.size();
}
```
Esta función template recibe como argumento un vector desordenado de interfaces y lo convierte en un vector ordenado. Es necesario incluir ordered_interfaces con referencias a las interfaces que coinciden, en el mismo orden que en joint_names.

### Método on_activate()

Se utiliza el método on_activate() para declarar un mensaje de comando y establecer el valor predeterminado para el comando (copiar y pegar este código como en casos anteriores).

```
CallbackReturn RRBotController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command
  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  msg->joint_names = joint_names_;
  msg->displacements.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
  input_command_.writeFromNonRT(msg);

  return CallbackReturn::SUCCESS;
}
```
#### Explicación del código 
```
  // Set default value in command
  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  msg->joint_names = joint_names_;
  msg->displacements.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
```
Arriba declaramos un mensaje utilizando el tipo de dato ControllerCommandMsg, se completa el campo joint_names en el mensaje y se establece el valor predeterminado en el comando al valor especial "quiet not-a-number", que tiene un significado específico para los tipos de punto flotante.
```
  input_command_.writeFromNonRT(msg);
```
Aquí se usa writeFromNonRT, que se puede utilizar en tiempo real (RT), si tenemos la garantía de que:

* Ningún hilo no-RT está llamando a la misma función (no estamos suscribiéndonos a callbacks de ROS).
* Solo hay un único hilo RT.

### Método on_deactivate()

Copiar y pegar el sigiente código que implementa el método on_deactivate:
```
CallbackReturn RRBotController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}
```
Lo único que se se hace aquí es devolver CallbackReturn::SUCCESS cuando se llame al método.

### Método update()

La función update() se llama en el bucle de control para generar un comando de control para el hardware. Para implementar el método update(), pega el siguiente código al final del archivo pero dentro del espacio de nombres:
```
controller_interface::return_type
RRBotController::update(const rclcpp::Time &time,
                        const rclcpp::Duration & /*period*/) {
  auto current_command = input_command_.readFromRT();

  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    if (!std::isnan((*current_command)->displacements[i])) {
        if (!command_interfaces_[i].set_value((*current_command)->displacements[i])){
            RCLCPP_ERROR(
                get_node()->get_logger(), 
                "Failed to set command interface value for joint %zu", 
                i
            );
            return controller_interface::return_type::ERROR;
        }
    }
  }

  if (state_publisher_ && state_publisher_->trylock()) {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = command_interfaces_[0].get_value();

    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}
```
#### Explicación del código 
```
auto current_command = input_command_.readFromRT();
```
Aquí se obtiene un puntero de datos con readFromRT()
```
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    if (!std::isnan((*current_command)->displacements[i])) {
        if (!command_interfaces_[i].set_value((*current_command)->displacements[i])){
            RCLCPP_ERROR(
                get_node()->get_logger(), 
                "Failed to set command interface value for joint %zu", 
                i
            );
            return controller_interface::return_type::ERROR;
        }
    }
  }
```
Para cada articulación, se establece como valor de la acción de control los datos no modificados que están dentro del vector de desplazamientos.
```
  if (state_publisher_ && state_publisher_->trylock()) {
    state_publisher_->msg_.header.stamp = get_node()->now();
    state_publisher_->msg_.set_point = command_interfaces_[0].get_value();

    state_publisher_->unlockAndPublish();
  }
```
Este ejemplo está implementando un controlador Forward command controller para un conjunto de articulaciones. Básicamente, reenvía como señal de control el valor pasado como consigna.

### macro PLUGINLIB_EXPORT_CLASS

Después de cerrar el espacio de nombres, hay que añadir una llamada a la macro PLUGINLIB_EXPORT_CLASS al final del archivo .cpp. Agregar el siguiente bloque de código al final de tu archivo de código fuente, después de que se cierre el espacio de nombres:
```
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rrbot_controller::RRBotController, controller_interface::ControllerInterface)
```
Esta macro sirve para registrar este controlador dentro del sistema de plugins de ROS2.

## Escribir un fichero de descripción del plugin

El siguiente paso es escribir un archivo de definición de exportación para Pluginlib, tal como se hace cuando se crea cualquier otro plugin de Pluginlib.
Crear un nuevo archivo llamado rrbot_controller.xml en el directorio raíz del paquete donde se encuentra el archivo CMakeLists.txt:
```
touch ~/ros2_ws/src/rrbot_controller/rrbot_controller.xml
```
Este será el contenido del xml:
```
<library path="rrbot_controller">
  <class name="rrbot_controller/RRBotController"
         type="rrbot_controller::RRBotController" base_class_type="controller_interface::ControllerInterface">
  <description>
    RRBotController ros2_control controller.
  </description>
  </class>
</library>
```
Guardar el contenido y seguir con el siguiente paso.

#### Explicación del código 

El elemento <library> especifica la ruta relativa a la biblioteca que contiene el complemento que se desea exportar. En este caso, es rrbot_controller.

La etiqueta <class...> declara el complemento que se quiere exportar. Los parámetros son los siguientes:

* name: nombre del complemento de ROS2_control.
* type: espacio de nombres y el nombre de la clase que implementa el complemento.
* base_class_type: espacio de nombres y nombre de la clase base de la que hereda este complemento.
La etiqueta anidada <description> encierra una descripción del complemento y su funcionalidad.

Nota: El archivo XML del complemento debe declarar todos los complementos contenidos en un paquete. En este caso, solo hay uno.

## Preparar los ficheros CMakeLists.txt y package.xml

### Añadir las directivas de compilación necesarias en el archivo CMakeLists.txt

Este paso agrega las directivas de compilación necesarias en CMakeLists.txt, que son requeridas para compilar el paquete.

El archivo CMakeLists.txt final debería ser el siguiente:
```
cmake_minimum_required(VERSION 3.5)
project(rrbot_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(example_interfaces REQUIRED)

add_library(
  rrbot_controller
  SHARED
  src/rrbot_controller.cpp
)
target_include_directories(
  rrbot_controller
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  rrbot_controller
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
# prevent pluginlib from using boost
target_compile_definitions(rrbot_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  controller_interface rrbot_controller.xml)

install(
  TARGETS
  rrbot_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  rrbot_controller
)
ament_export_dependencies(
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

ament_package()
```

#### Explicación del código 
```
add_library(
  rrbot_controller
  SHARED
  src/rrbot_controller.cpp
)

target_include_directories(
  rrbot_controller
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  rrbot_controller
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
```
Aquí se agrega los archivos fuente al comando add_library() de CMake. También, hay que tener en cuenta el uso de expresiones generadoras con la sintaxis $<.. : ..> como argumentos para la directiva target_include_directories.
```
target_compile_definitions(rrbot_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
```
Esta línea evita que pluginlib use Boost.
```
pluginlib_export_plugin_description_file(controller_interface rrbot_controller.xml)
```
El comando de CMake anterior instalará el archivo de descripción del complemento (rrbot_controller.xml) para que pluginlib pueda cargar el complemento.

Los argumentos de este comando son:

* El paquete de la clase base, es decir, controller_interface.
* La ruta relativa al archivo XML de declaración del complemento, en este caso, solo el nombre del archivo: rrbot_controller.xml.
```
install(
  TARGETS
  rrbot_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)
```
Esto copiará los binarios generados en lib e include. También indicamos a CMake que instale todos los archivos de lanzamiento dentro de la carpeta launch.
```
ament_export_include_directories(
  include
)
ament_export_libraries(
  rrbot_controller
)
ament_export_dependencies(
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
```
Las líneas anteriores exportan el directorio de inclusión, la biblioteca y las dependencias para que otros proyectos las utilicen.

## Añadir las dependencias en el paquete package.xml

Normalmente, se necesita agregar las dependencias externas al archivo package.xml para que ament pueda compilar el paquete.

Deja este archivo sin modificar, ya que en este controlador personalizado no se utiliza ninguna biblioteca externa.
```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rrbot_controller</name>
  <version>0.0.0</version>

  <description>Controller for exemplary RRBot robot.</description>

  <maintainer email="bence.magyar.robotics@gmail.com">Bence Magyar</maintainer>
  <maintainer email="denis@stogl.de">Denis Štogl</maintainer>

  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>control_msgs</depend>
  <depend>controller_interface</depend>
  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_lifecycle</depend>
  <depend>realtime_tools</depend>
  <depend>example_interfaces</depend>

  <test_depend>ament_cmake_gmock</test_depend>
  <test_depend>controller_manager</test_depend>
  <test_depend>hardware_interface</test_depend>
  <test_depend>ros2_control_test_assets</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
Ahora se debería compilar y corregir cualquier error que pueda aparecer:
```
cd ~/ros2_ws && colcon build --packages-select rrbot_controller
```
## Crear un fichero de configuración del controlador para el Controller Manager (.yaml)
Ahora se van a crear los archivos de configuración para ros2_control y el controlador que se utilizará. El archivo de configuración estará dentro de la carpeta config, del paquete my_robot_bringup creado anteriormente en la práctica.
```
touch ~/ros2_ws/src/my_robot_bringup/config/rrbot_controllers_custom.yaml
```
Copiar el siguiente código en el fichero rrbot_controllers_custom.yaml:
```
# Controller manager configuration
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

    # Define a name for controllers that we plan to use
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    rrbot_controller:
      type: rrbot_controller/RRBotController

# Properties of the custom controler and definition of joints to use
rrbot_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
    interface_name: position
```
En este archivo de configuración se establece que se utilizará el nuevo controlador para las dos articulaciones del robot. 

## Actualizar los parámetros de configuración del plugin de Gazebo

Ahora se necesita cargar el nuevo archivo de configuración YAML. Al trabajar con Gazebo, se debe referenciar este archivo dentro de los parámetros de configuración del plugin de Gazebo. Para ello, abrir el archivo XACRO del robot (rrbot.gazebo.xacro). En concreto, el archivo debería encontrarse aquí: ~/ros2_ws/src/ros2_robot_sca/description/gazebo.

Localizar los tags del plugin de Gazebo ros2_control y modificar el elemento correspondiente para usar el nuevo fichero de configuración rrbot_controllers_custom.yaml, en lugar del que se está usando actualmente.

Esto es lo que debe aparecer en el nuevo plugin de configuración de Gazebo ros2_control:

```
<!-- ros_control plugin -->
<gazebo>
  <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(find my_robot_bringup)/config/rrbot_controllers_custom.yaml</parameters>
  </plugin>
</gazebo>
```

## Crear un nuevo archivo de lanzamiento para generar el robot y ejecutar el nuevo controlador

Crear un nuevo archivo de lanzamiento es opcional, ya que es posible iniciar y detener controladores únicamente utilizando la interfaz de línea de comandos del controller manager. Sin embargo, es más cómodo crear un nuevo archivo de lanzamiento que genere el robot en Gazebo y ejecute el nuevo controlador. Este nuevo fichero de lanzamiento se creará en la carpeta launch del paquete my_robot_bringup creado en la práctica:
```
touch ~/ros2_ws/src/my_robot_bringup/launch/rrbot_with_rrbot_controller.launch.py
```
El contenido de este fichero de lanzamiento será el siguiente:
``` py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": " -r -v 3 empty.sdf"}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "rrbot_system_position",
            "-allow_renaming",
            "true",
        ],
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ros2_robot_sca"), "urdf", "rrbot.urdf.xacro"]
            ),
            " ",
            "use_gazebo:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("my_robot_bringup"),
            "config",
            "rrbot_controllers_custom.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_robot_sca"), "rrbot/rviz", "rrbot.rviz"]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rrbot_controller", "--param-file", robot_controllers],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    nodes = [
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
```

El fichero launch es parecido al creado anteriormente en la práctica, la única diferencia es este código:
```
robot_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["rrbot_controller", "--param-file", robot_controllers],
)
```
La diferencia es que el archivo de lanzamiento iniciará un controlador con el nombre rrbot_controller.

## Recompilar y probar el nuevo controlador

Ahora se debe recompilar y ejecutar source antes de continuar y probar el controlador en el robot. Es necesario recompilar para que el nuevo archivo de lanzamiento y el archivo de configuración se copien en el espacio de instalación.
```
cd ~/ros2_ws
```

```
colcon build
```

Si seguiste los pasos uno a uno hasta aquí, verás que el código se compila correctamente.
```
source install/setup.bash
```

Ejecuta el nuevo archivo de lanzamiento que genera el robot y inicia el nuevo controlador:
```
ros2 launch my_robot_bringup rrbot_with_rrbot_controller.launch.py
```
En otra terminar confirmar que el nuevo controlador está activo (debe aparece el controlador rrbot_controller):
```
ros2 control list_controllers
```
En ocasiones es posible que en ROS 2 Jazzy, cuando creas un nuevo controlador como plugin para ros2_control, el sistema no lo detecte automáticamente hasta que reinicies el ordenador o máquina virtual. Este comportamiento ocurre porque ros2_control carga los plugins disponibles al iniciar el sistema, y no siempre detecta dinámicamente los nuevos plugins que se añaden después. 

A continuación probar a mover las articulaciones del robot utilizando el nuevo controlador:
```
ros2 topic pub /rrbot_controller/commands control_msgs/msg/JointJog "{joint_names: ['joint_1', 'joint_2'], displacements: [0.5, 0.3]}"
```


## Implementación de un controlador PD con compensación de gravedad

En este apartado se va a modificar el controlador desarrollado en el apartado anterior para implementar un controlador PD con compensación de gravedad. La acción de control de un controlador PD con compensación de gravedad es la siguiente:

$$
\tau= K_{p}\tilde{q}+K_{v}\dot{\tilde{q}}+q(q)
$$

donde $K_{p}$ y $K_{p}$ son matrices diagonales porporcional y derivativa respectivamente de tamaño nxn siendo n el número de grados de libertad del robot, $\tilde{q}=q_d-q(t)$ es el error entre la configuración articular a alcanzar y la posición articular actual, y $g(q)$ es la componente de gravedad del modelo dinaḿico del robot evaluada en la posición actual.

Para implementarlo se pueden seguir los pasos que se indican en los siguientes párrafos. Primero será necesario incluir la libreria Pinocchio en el fichero rrbot_controller.cpp. Esta librería se utilizará para acceder al vector de gravedad del modelo dinámico del robot:

```
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
```

A continuación, modificar la funcion on_init para leer los parámetros de los que depende el controlador PD con compensación de gravedad (ganancias proporcional y derivativa):
```
CallbackReturn RRBotController::on_init() {
    try {
      auto_declare("joints", std::vector<std::string>());
      auto_declare("interface_name", std::string());
      
      // Parámetros para el controlador PD con compensación de gravedad
      auto_declare("kp", std::vector<double>());
      auto_declare("kd", std::vector<double>());
      
      // Parámetro para la ruta del archivo URDF
      auto_declare("robot_description_path", std::string());
    } catch (const std::exception &e) {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n",
              e.what());
      return CallbackReturn::ERROR;
    }
  
    return CallbackReturn::SUCCESS;
  }
```

A continuación, introducir el siguiente código en On_configure justo después de obtener joints y el interface_name y antes de crear el Command Subscriber and callbacks. Esto nos va a permitir crear el modelo de pinocchio:
```
  // Obtener ganancias Kp y Kd
  kp_ = get_node()->get_parameter("kp").as_double_array();
  kd_ = get_node()->get_parameter("kd").as_double_array();
  
  // Verificar que tenemos el número correcto de ganancias
  if (kp_.size() != joint_names_.size() || kd_.size() != joint_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), 
                "Se esperaban %zu ganancias, pero se recibieron %zu Kp y %zu Kd",
                joint_names_.size(), kp_.size(), kd_.size());
    return CallbackReturn::ERROR;
  }
  
  // Obtener la ruta del archivo URDF
  std::string urdf_path;
  get_string_param_and_error_if_empty(urdf_path, "robot_description_path");
  
  // Inicializar el modelo de Pinocchio desde el URDF
  try {
    // Crear el modelo y los datos
    model_ = std::make_shared<pinocchio::Model>();
    
    // Cargar el modelo desde el URDF
    pinocchio::urdf::buildModel(urdf_path, *model_);
    data_ = std::make_shared<pinocchio::Data>(*model_);
    
    // Verificar que los nombres de las articulaciones coinciden
    bool joints_found = true;
    joint_indices_.resize(joint_names_.size());
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (model_->existJointName(joint_names_[i])) {
        joint_indices_[i] = model_->getJointId(joint_names_[i]);
      } else {
        joints_found = false;
        RCLCPP_ERROR(get_node()->get_logger(), 
                    "No se encontró la articulación '%s' en el modelo URDF",
                    joint_names_[i].c_str());
      }
    }
    
    if (!joints_found) {
      return CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "Modelo Pinocchio inicializado con éxito");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), 
                "Error al inicializar el modelo Pinocchio: %s", 
                e.what());
    return CallbackReturn::ERROR;
  }
```

En controller_interface::InterfaceConfiguration RRBotController::command_interface_configuration() cambiar el interfaz para controlar por torque:

```
command_interfaces_config.names.push_back(joint + "/effort");  // Cambiado a effort para control de torque
```


En controller_interface::InterfaceConfiguration RRBotController::state_interface_configuration cambiarla por la siguiente función ya que necesitamos leer tanto la posición como la velocidad articular para implementar el control PD:

```
controller_interface::InterfaceConfiguration RRBotController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Necesitamos leer tanto la posición como la velocidad para el control PD
  state_interfaces_config.names.reserve(joint_names_.size() * 2);
  for (const auto & joint : joint_names_) {
    state_interfaces_config.names.push_back(joint + "/position");
    state_interfaces_config.names.push_back(joint + "/velocity");
  }

  return state_interfaces_config;
}
```

Antes de get_ordered interfaces crear esta función para calcular el vector de gravedad usando Pinocchio:
```
std::vector<double> RRBotController::calculate_gravity_vector(const std::vector<double>& q)
{
  // Crear un vector de configuración para Pinocchio (podría incluir articulaciones base flotante)
  Eigen::VectorXd q_pin = Eigen::VectorXd::Zero(model_->nq);
  
  // Llenar el vector de configuración con los valores de las articulaciones controladas
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    // Ajustar el índice según cómo esté estructurado tu modelo en Pinocchio
    q_pin[i] = q[i];
  }
  
  // Velocidades y aceleraciones cero para calcular solo la gravedad
  Eigen::VectorXd v_pin = Eigen::VectorXd::Zero(model_->nv);
  Eigen::VectorXd a_pin = Eigen::VectorXd::Zero(model_->nv);
  
  // Calcular la dinámica inversa con velocidades y aceleraciones cero para obtener solo el vector de gravedad
  pinocchio::rnea(*model_, *data_, q_pin, v_pin, a_pin);
  
  // Extraer el torque de gravedad para nuestras articulaciones
  std::vector<double> g_vector(joint_names_.size(), 0.0);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    g_vector[i] = data_->tau[model_->joints[joint_indices_[i]].idx_v()];
  }
  
  return g_vector;
}
```

En On_activate incorporar el siguiente código que permitirá leer posición y velocidad articular actual:
```
// Obtener las interfaces de estado ordenadas (posición y velocidad)
  position_state_interfaces_.clear();
  velocity_state_interfaces_.clear();
  
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    // Buscar las interfaces de posición
    for (auto& interface : state_interfaces_) {
      if (interface.get_name() == (joint_names_[i]+"/"+interface.get_interface_name()) && 
          interface.get_interface_name() == "position") {
        position_state_interfaces_.push_back(std::ref(interface));
        break;
      }
    }
    
    // Buscar las interfaces de velocidad
    for (auto& interface : state_interfaces_) {
      if (interface.get_name() == (joint_names_[i]+"/"+interface.get_interface_name()) && 
          interface.get_interface_name() == "velocity") {
        velocity_state_interfaces_.push_back(std::ref(interface));
        break;
      }
    }
  }
  
  // Verificar que tenemos todas las interfaces necesarias
  if (position_state_interfaces_.size() != joint_names_.size() ||
      velocity_state_interfaces_.size() != joint_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), 
                "No se pudieron encontrar todas las interfaces de estado necesarias");
    return CallbackReturn::ERROR;
  }
```

La nueva función update que calcula la acción de control será la siguiente:
```
controller_interface::return_type
RRBotController::update(const rclcpp::Time &time,
                        const rclcpp::Duration &period) {
  auto current_command = input_command_.readFromRT();
  
  // Obtener las posiciones y velocidades actuales
  std::vector<double> current_positions(joint_names_.size(), 0.0);
  std::vector<double> current_velocities(joint_names_.size(), 0.0);
  
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    current_positions[i] = position_state_interfaces_[i].get().get_value();
    current_velocities[i] = velocity_state_interfaces_[i].get().get_value();
  }
  
  // Obtener posiciones deseadas del comando
  std::vector<double> desired_positions(joint_names_.size(), 0.0);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (!std::isnan((*current_command)->displacements[i])) {
      desired_positions[i] = (*current_command)->displacements[i];
    } else {
      desired_positions[i] = current_positions[i];  // Mantener posición actual si no hay comando
    }
  }
  
  // Calcular el vector de gravedad usando Pinocchio
  std::vector<double> gravity = calculate_gravity_vector(current_positions);

  // Calcular el error de posición y velocidad
  std::vector<double> position_error(joint_names_.size(), 0.0);
  
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    position_error[i] = desired_positions[i] - current_positions[i];
  }
  
  // Calcular el comando de torque usando PD con compensación de gravedad
  // tau = Kp * (q_d - q) - Kd * q_dot + g(q)
  std::vector<double> torque_command(joint_names_.size(), 0.0);
  
  for (size_t i = 0; i < joint_names_.size(); ++i) {
     torque_command[i] = kp_[i] * position_error[i] - 
                      kd_[i] * current_velocities[i] + 
                      gravity[i];
  }
  
  // Aplicar el comando de torque
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (!command_interfaces_[i].set_value(torque_command[i])) {
      RCLCPP_ERROR(
          get_node()->get_logger(), 
          "Failed to set command interface value for joint %zu", 
          i
      );
      return controller_interface::return_type::ERROR;
    }
  }

  if (state_publisher_ && state_publisher_->trylock()) {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = desired_positions[0];  // Publicar la posición deseada

    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}
```

También será necesario modificar el fichero de encabezado rrbot_controller.hpp para incluir las dependencias:
```
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <hardware_interface/loaned_state_interface.hpp>
```
La sección protected de este fichero sería la siguiente:
```
std::vector<double> calculate_gravity_vector(const std::vector<double>& q);
  
  std::vector<std::string> joint_names_;
  std::string interface_name_;

  // Command subscribers and Controller State publisher
  using ControllerCommandMsg = control_msgs::msg::JointJog;

  // Command subscribers and Controller State publisher
  using ControllerStateMsg = control_msgs::msg::JointControllerState;
  using ControllerStatePublisher =
  realtime_tools::RealtimePublisher<ControllerStateMsg>;

  // Parámetros del controlador PD
  std::vector<double> kp_;  // Ganancia proporcional
  std::vector<double> kd_;  // Ganancia derivativa

  // Modelo y datos de Pinocchio
  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;
  std::vector<pinocchio::JointIndex> joint_indices_;  // Índices de las articulaciones en el modelo

  // Interfaces de estado organizadas
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> position_state_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> velocity_state_interfaces_;

  // Subscriber para comandos y buffer
  rclcpp::Subscription<ControllerCommandMsg>::SharedPtr command_subscriber_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandMsg>> input_command_;

  // Publisher para estado
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;
```
Además, es necesario actualizar el archivo YAML rrbot_controllers_custom.yaml para incluir las ganancias de los controladores y la ruta del archivo URDF:
```
# Controller manager configuration
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz
    # Define a name for controllers that we plan to use
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    rrbot_controller:
      type: rrbot_controller/RRBotController

# Properties of the custom controler and definition of joints to use
rrbot_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
    interface_name: position  # Se mantiene por compatibilidad
    
    # Ganancias del controlador PD
    kp: [100.0, 100.0]  # Ganancia proporcional para cada articulación
    kd: [10.0, 10.0]    # Ganancia derivativa para cada articulación
    
    # Ruta del archivo URDF para Pinocchio
    robot_description_path: "/home/ubuntu/ros2_ws/src/ros2_robot_sca/description/urdf/model.urdf"
```

Antes de poder compilar todo será necesario instalar pinocchio:
```
sudo apt-get install ros-jazzy-pinocchio
```
También es necesario actualizar las dependencias del paquete rrbot_controller. Para ello, se debe añadir Pinocchio como dependencia en el archivo package.xml:
```
<depend>pinocchio</depend>
```
Y CMakeLists.txt:
```
find_package(pinocchio REQUIRED)
```
En el CMakeLists, añadir también pinocchio en ament_target_dependencies y en ament_export_dependencies.

Puede que también necesites incluir en CMakeLists los cambios necesarios debido a que pinocchio utiliza Eigen3. Para ello, primero añadir el siguiente código:
```
find_package(Eigen3 REQUIRED NO_MODULE)
```
Luego, asegúrate de incluir los directorios de Eigen3 correctamente:
```
include_directories(${EIGEN3_INCLUDE_DIRS})
```
Y finalmente, añade la dependencia a tu target:
```
target_link_libraries(rrbot_controller ${EIGEN3_LIBRARIES})
```

### Recompilar y probar el nuevo controlador

Ahora se debe recompilar y ejecutar source antes de continuar y probar el controlador en el robot. Es necesario recompilar para que el nuevo archivo de lanzamiento y el archivo de configuración se copien en el espacio de instalación.
```
cd ~/ros2_ws
```

```
colcon build
```
Puede que obtengas bastantes warnings de Eigen y Pinocchio sobre posibles variables no inicializadas en los constructores. Estas advertencias no deberían impedir que tu controlador se compile correctamente. 

```
source install/setup.bash
```
Ejecuta el nuevo archivo de lanzamiento que genera el robot e inicia el nuevo controlador:
```
ros2 launch my_robot_bringup rrbot_with_rrbot_controller.launch.py
```
En otra terminal confirmar que el nuevo controlador está activo (debe aparece el controlador rrbot_controller):
```
ros2 control list_controllers
```
A continuación probar a mover las articulaciones del robot utilizando el nuevo controlador:
```
ros2 topic pub /rrbot_controller/commands control_msgs/msg/JointJog "{joint_names: ['joint_1', 'joint_2'], displacements: [0.5, 0.3]}"
```

!!! note annotate "Ejercicio"

    * Ajustar el controlador PD con compensación de gravedad.
    * Comparar el comportamiento del controlador PD con compensación de gravedad frente a un controlador PD.
    * Implementar un controlador PID y ajustarlo correctamente.
    * Comparar el comportamiento de los tres controladores: PD, PD con compensación de gravedad y PID.
