//| This file is a part of the CAFER framework developped within
//| the DREAM project (http://www.robotsthatdream.eu/).
//|
//| Copyright 2015, GII / Universidad de la Coruna (UDC)
//| Main contributor(s): 
//|   * Rodrigo Salgado, rodrigo.salgado@udc.es
//|   * Pilar Caamano, pilar.caamano@udc.es
//|   * Juan Monroy, juan.monroy@udc.es
//|   * Luis Calvo, luis.calvo@udc.es
//|   * Jose Antonio Becerra, jose.antonio.becerra.permuy@udc.es
//|
//| This file is also part of MDB.
//| 
//| * MDB is free software: you can redistribute it and/or modify it under the
//| * terms of the GNU Affero General Public License as published by the Free
//| * Software Foundation, either version 3 of the License, or (at your option) any
//| * later version.
//| *
//| * MDB is distributed in the hope that it will be useful, but WITHOUT ANY
//| * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
//| * A PARTICULAR PURPOSE. See the GNU Affero General Public License for more
//| * details.
//| *
//| * You should have received a copy of the GNU Affero General Public License
//| * along with MDB. If not, see <http://www.gnu.org/licenses/>.

package es.udc.gii.mdb.ros;

import es.udc.gii.mdb.util.testing.view.RobotSimulator;
import es.udc.gii.mdb.util.xml.ConfigUtilXML;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.MalformedURLException;
import java.net.Socket;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.apache.commons.configuration.Configuration;
import org.apache.commons.configuration.ConfigurationException;
import org.apache.commons.configuration.XMLConfiguration;
import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import org.ros.node.parameter.ParameterTree;
import org.ros.namespace.NameResolver;
import org.ros.message.MessageListener;
import java.lang.IllegalStateException;
import org.ros.exception.RosRuntimeException;
import org.ros.node.Node;
import java.lang.InterruptedException;

/**
 * A simple {@link Publisher} {@link NodeMain}.
 */
public class ROSSimulation extends AbstractNodeMain {

    private ConnectedNode connectedNode;
    private HashMap<String, Publisher> sensorPublishers;
    private HashMap<String, Subscriber> actuatorSubscribers;
    private HashMap<String, DataInputStream> sensorSockets;
    private HashMap<String, DataOutputStream> actuatorSockets;
    private CancellableLoop cp;
    private Log log;
        
    private String PATH = "";
    private boolean running;
    
    public boolean getRunning(){
        return running;
    }
    
    public ROSSimulation() {
        super();
        sensorPublishers = new HashMap<>();
        actuatorSubscribers = new HashMap<>();
        sensorSockets = new HashMap<>();
        actuatorSockets = new HashMap<>();

    }
    
    private String ObtainPath(ConnectedNode cn){
        ParameterTree params = cn.getParameterTree();
        NameResolver namer = cn.getResolver();
        GraphName namespace = namer.getNamespace();
        String path_param = namespace.toString()+"/rosmdb_path";
        boolean has_path = params.has(path_param);
        String path = "";
        if (has_path==true) {
            path = params.getString(path_param)+"/";
        }
        return path;
    }
    
    public ConnectedNode getNode(){
        return connectedNode;
    }

    
    ///ROSSimulation:
    ///Sensores: Publishers.
    ///Actuadores: Listeners.
    private void createProxy() throws ConfigurationException, MalformedURLException, InterruptedException, IOException {
        //para cada sensor en la configuración
        //  creo un publisher (nombre del sensor)
        //  creo un socket servidor para obtener la medida del simulador

        Configuration configuration = new XMLConfiguration(PATH + "__door/MDB-Config-sim.xml");
        Configuration sensorsConf = configuration.subset(ConfigUtilXML.SENSORS_GROUP_TAG);
        
            ArrayList<String> names = new ArrayList<String>();
            for (Object o : sensorsConf.getList("sensor.id")) {
                String sensorName = (String) o;
                Publisher<Object> newPublisher = connectedNode.newPublisher(sensorName, std_msgs.String._TYPE);
                newPublisher.setLatchMode(true);
                sensorPublishers.put(sensorName, newPublisher);
                names.add(sensorName);
                System.out.println("Añadido publish sensor " + sensorName);
            }
            int i = 0;
            for (Object o : sensorsConf.getList("sensor.port")) {
                Integer port = new Integer((String) o);
                System.out.println("Intentando conexion a puerto " + port + "para sensor " + names.get(i));
                Socket socket = new Socket("localhost", port);
                Thread.sleep(1000);
                DataInputStream dis = new DataInputStream(socket.getInputStream());
                sensorSockets.put(names.get(i), dis);
                System.out.println("Añadido socket sensor " + names.get(i));
                i++;
            }
            
            names = new ArrayList<String>();
            Configuration actsConf = configuration.subset(ConfigUtilXML.ACTUATORS_GROUP_TAG);
            for (Object o : actsConf.getList("actuator.id")) {
                String actName = (String) o;
                Subscriber sbs = connectedNode.newSubscriber(actName, std_msgs.String._TYPE);
                sbs.addMessageListener(new IdentifiedMessageListener<std_msgs.String>(actName) { 
                    @Override
                    public void onNewMessage(std_msgs.String msg) {
                        double readed = new Double(msg.getData());
                        log.info("Recibido para actuador" + getId() + " " + readed + " (enviando por socket)");
                        log.debug("Recibido para actuador" + getId() + " " + readed + " (enviando por socket)");
                        
                        System.out.println("Recibido para actuador" + getId() + " " + readed + " (enviando por socket)");
                        try {
                            while (actuatorSockets.get(getId())==null); //esperamos a que el socket esté creado
                            actuatorSockets.get(getId()).writeDouble(readed);
                            System.out.println("\n\n\n\n\n\n\n");
                        } catch (IOException ex) {
                            Logger.getLogger(ROSSimulation.class.getName()).log(Level.SEVERE, null, ex);
                        }
                    }
                },10);
                actuatorSubscribers.put(actName, sbs);
                names.add(actName);
                System.out.println("Añadido subscript actuador " + actName);
            }
            i = 0;
            for (Object o : actsConf.getList("actuator.port")) {
                Integer port = new Integer((String) o);
                Socket socket = new Socket("localhost", port);
                Thread.sleep(1000);
                DataOutputStream dos = new DataOutputStream(socket.getOutputStream());
                actuatorSockets.put(names.get(i), dos);
                System.out.println("Añadido socket actuador " + names.get(i));
                i++;
            }
    }

    private void launchSimulator() {
		running = true;
        Thread simThread = new Thread(new Runnable() {
            @Override
            public void run() {
                RobotSimulator.main(new String[]{"-x", "-mdb", PATH + "__door/CoinCollectorSimple-Config.xml", PATH + "__door/MDB-Config-sim.xml"});
			}
        });
        simThread.start();
        log.info("¡Simulador lanzado!");
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rossimulation");
    }
    
    private void closeSockets (){
        Set<Map.Entry<String, DataInputStream>> entrySetdis = sensorSockets.entrySet();
        for (final Map.Entry<String, DataInputStream> entry : entrySetdis) {
            DataInputStream dis = entry.getValue();
            try{
                dis.close();
            } catch (IOException ex) {
                Logger.getLogger(ROSSimulation.class.getName()).log(Level.SEVERE, null, ex);
                System.out.println(ex);
            }
        }
        
        Set<Map.Entry<String, DataOutputStream>> entrySetaos = actuatorSockets.entrySet();
        for (final Map.Entry<String, DataOutputStream> entry : entrySetaos) {
            DataOutputStream dos = entry.getValue();
            try{
                dos.close();
            } catch (IOException ex) {
                Logger.getLogger(ROSSimulation.class.getName()).log(Level.SEVERE, null, ex);
                System.out.println(ex);
            }
        }
    }
    
    private void cleanNode (){
        Set<Map.Entry<String, Subscriber>> entrySet_as = actuatorSubscribers.entrySet();
        for (final Map.Entry<String, Subscriber> entry : entrySet_as) {
            Subscriber sub = entry.getValue();
            sub.shutdown();
        }
        Set<Map.Entry<String, Publisher>> entrySet_sp = sensorPublishers.entrySet();
        for (final Map.Entry<String, Publisher> entry : entrySet_sp) {
            Publisher pub = entry.getValue();
            pub.shutdown();
        }
    }
    
    /*@Override
    public void onShutdown(Node node) {
        //System.exit(0);
        System.out.println("\nROSSimulation_on_shutdown");
    }*/
    
    @Override
    public void onShutdownComplete(Node node) {
        System.out.println("\nROSSimulation_on_shutdown_complete");
        System.exit(0);
    }
    
    /*@Override
    public void onError(Node node, Throwable throwable) {
        System.out.println("\nROSSimulation_on_error");
    }*/

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        connectedNode.getLog().info("LOG ACTIVADO "+connectedNode.getLog().isInfoEnabled());
        final Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber("rosmdb_topic", std_msgs.String._TYPE);
        try {
            this.connectedNode = connectedNode;
            log = connectedNode.getLog();
            this.PATH = ObtainPath(connectedNode);
            System.out.println("Lanzando simulador...");
            launchSimulator();
            log.info("Arrancando simulador");
            Thread.sleep(5000);
            System.out.println("Creando proxy...\n");
            createProxy();
            System.out.println("Iniciando bucle principal...");
            threadLoop();

        } catch (ConfigurationException ex) {
            System.out.println(ex);
        } catch (InterruptedException ex) {
            Logger.getLogger(ROSSimulation.class.getName()).log(Level.SEVERE, null, ex);
            System.out.println(ex);
        } catch (IOException ex) {
            Logger.getLogger(ROSSimulation.class.getName()).log(Level.SEVERE, null, ex);
            System.out.println(ex);
        }
        
        subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
            @Override
            public void onNewMessage(std_msgs.String message) {
                String option = message.getData();
                switch (option){
                    case "stop":
                        try{
                            System.out.println("\n\nCerrando ROSSimulation\n\n");
                            boolean run = getRunning();
                            run = false;
                            subscriber.shutdown();
                            cleanNode();
                            ConnectedNode cn = getNode();
                            cn.shutdown();
                            break;
                        }catch(IllegalStateException ise){
                            System.out.println(ise);
                        }catch(RosRuntimeException rre){
                            System.out.println("rre"+rre);
                        }
                    default:
                        break;
                }
            }
        });

    }

    private void threadLoop() {
        running = true;
        Set<Map.Entry<String, DataInputStream>> entrySet = sensorSockets.entrySet();
        for (final Map.Entry<String, DataInputStream> entry : entrySet) {
            final String name = entry.getKey();
            Thread t = new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        while (running) {
                            System.out.println("Esperando a recibir de: " + name);
                            double readed = entry.getValue().readDouble();
                            log.info("Recibido de sensor " + name + " " + readed + " (publicando por ros)");
                            System.out.println("Recibido de sensor " + name + " " + readed + " (publicando por ros)");
                            std_msgs.String str = (std_msgs.String) sensorPublishers.get(name).newMessage();
                            str.setData("" + readed);
                            sensorPublishers.get(name).publish(str);
                        }
                    } catch (IOException ex) {
                        Logger.getLogger(ROSSimulation.class.getName()).log(Level.SEVERE, null, ex);
                    }

                }
            });
            
            t.setDaemon(true);
            t.start();
        }
    }
}
