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

import es.udc.gii.mdb.core.MainCore;
import es.udc.gii.mdb.util.testing.robot.actuator.MDBRobotActuator;
import es.udc.gii.mdb.util.testing.robot.sensor.MDBRobotSensor;
import es.udc.gii.mdb.util.testing.robot.socket.SocketReader;
import es.udc.gii.mdb.util.testing.robot.socket.SocketWriter;
import es.udc.gii.mdb.util.xml.ConfigUtilXML;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.apache.commons.configuration.Configuration;
import org.apache.commons.configuration.ConfigurationException;
import org.apache.commons.configuration.XMLConfiguration;
import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;

import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;

import org.ros.node.parameter.ParameterTree;
import org.ros.namespace.NameResolver;
import java.lang.IllegalStateException;
import org.ros.exception.RosRuntimeException;
import org.ros.node.Node;
import java.util.Map;
import java.util.Set;
import org.ros.node.NodeMainExecutor;
import java.lang.InterruptedException;
import java.lang.NullPointerException;


/**
 * A simple {@link Subscriber} {@link NodeMain}.
 */
public class ROSMDB extends AbstractNodeMain {

    private ConnectedNode connectedNode;
    private HashMap<String, Subscriber> sensorSubscribers;
    private HashMap<String, Publisher> actuatorPublishers;
    private HashMap<String, SocketWriter> sensorSockets;
    private HashMap<String, SocketReader> actuatorSockets;
    private CancellableLoop cp;
    private Log log;
    
    private boolean running;
    
    public boolean getRunning(){
        return running;
    }

    private String PATH = "";
    
    public ROSMDB() {
        super();
        sensorSubscribers = new HashMap<>();
        actuatorPublishers = new HashMap<>();
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

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rosmdb");
    }

    private void launchMDB() {
		running = true;
        Thread simThread = new Thread(new Runnable() {
            @Override
            public void run() {
               	MainCore.main(new String[]{PATH + "__door/MDB-Config.xml"});
            }
        });
        simThread.start();
    }	

    
    /// ROSMDB:
    /// Sensores: Suscribers
    /// Actuadores: Publishers
    private void createProxy() throws ConfigurationException {
        //para cada sensor en la configuración
        //  creo un subscriber (nombre del sensor)
        //  creo un sensor para darle el valor publicado al mdb

        Configuration configuration = new XMLConfiguration(PATH + "__door/MDB-Config.xml");
        Configuration sensorsConf = configuration.subset(ConfigUtilXML.SENSORS_GROUP_TAG);

            int i = 0;
            List<String> names = sensorsConf.getList("sensor.id");
            for (Object o : sensorsConf.getList("sensor.port")) {
                final Integer port = new Integer((String) o);
                System.out.println("Lanzando socket en puerto " + port + "para sensor " + names.get(i));
                final SocketWriter socket = new SocketWriter(names.get(i));
                System.out.println("Inicializando socket");
                socket.init(port, 0, 0, 0, 0);
                sensorSockets.put(names.get(i), socket);
                System.out.println("Añadido socket sensor " + names.get(i));
                i++;
            }

            for (Object o : sensorsConf.getList("sensor.id")) {
                String sensorName = (String) o;
                Subscriber sbs = connectedNode.newSubscriber(sensorName, std_msgs.String._TYPE);
                sbs.addMessageListener(new IdentifiedMessageListener<std_msgs.String>(sensorName) {
                    @Override
                    public void onNewMessage(std_msgs.String msg) {
                        double readed = new Double(msg.getData());
                        log.info("Recibido via ROS para sensor" + getId() + " " + readed + " (enviando por socket)");
                        sensorSockets.get(getId()).writeDouble(readed);
                    }
                },10);
                sensorSubscribers.put(sensorName, sbs);
            }

            names = new ArrayList<String>();
            Configuration actsConf = configuration.subset(ConfigUtilXML.ACTUATORS_GROUP_TAG);
            for (Object o : actsConf.getList("actuator.id")) {
                String actName = (String) o;
                Publisher<Object> newPublisher = connectedNode.newPublisher(actName, std_msgs.String._TYPE);
                newPublisher.setLatchMode(true);
                actuatorPublishers.put(actName, newPublisher);
                names.add(actName);
                System.out.println("Añadido publish actuador " + actName);
            }
            i = 0;
            for (Object o : actsConf.getList("actuator.port")) {
                final Integer port = new Integer((String) o);
                final SocketReader socket = new SocketReader(names.get(i));
                System.out.println("Inicializando socket");
                socket.init(port, 0, 0, 0, 0);
                actuatorSockets.put(names.get(i), socket);
                System.out.println("Añadido socket actuador " + names.get(i));
                i++;
            }
    }
    
    /*@Override
    public void onShutdown(Node node) {
        //System.exit(0);
        System.out.println("\nROSMDB_on_shutdown");
    }*/
    
    @Override
    public void onShutdownComplete(Node node) {
        System.out.println("\nROSMDB_on_shutdown_complete");
        System.exit(0);
    }
    
    /*@Override
    public void onError(Node node, Throwable throwable) {
        System.out.println("\nROSMDB_on_error");
    }*/
    
    private void closeSockets (){
        Set<Map.Entry<String, SocketWriter>> entrySetss = sensorSockets.entrySet();
        for (final Map.Entry<String, SocketWriter> entry : entrySetss) {
            SocketWriter sw = entry.getValue();
            try{
                sw.getDataOutputStream().close();
            }catch (IOException ioe){
                System.out.println(ioe);
            }
        }
        
        /*Set<Map.Entry<String, SocketReader>> entrySetas = actuatorSockets.entrySet();
        for (final Map.Entry<String, SocketReader> entry : entrySetas) {
            SocketReader sr = entry.getValue();
            sr.close();
        }*/
    }
    
    private void cleanNode (){
        Set<Map.Entry<String, Subscriber>> entrySet_ss = sensorSubscribers.entrySet();
        for (final Map.Entry<String, Subscriber> entry : entrySet_ss) {
             Subscriber sub = entry.getValue();
             sub.shutdown();
        }
        Set<Map.Entry<String, Publisher>> entrySet_ap = actuatorPublishers.entrySet();
        for (final Map.Entry<String, Publisher> entry : entrySet_ap) {
            Publisher pub = entry.getValue();
            pub.shutdown();
        }
    }
        

    @Override
    public void onStart(ConnectedNode connectedNode) {
        final Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber("rosmdb_topic", std_msgs.String._TYPE);
        try {
            Locale.setDefault(Locale.US);
            log = connectedNode.getLog();
            this.connectedNode = connectedNode;
            this.PATH = ObtainPath(connectedNode);
            System.out.println("Iniciando proxy...");
            Thread t = new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        createProxy();
                    } catch (ConfigurationException ex) {
                        Logger.getLogger(ROSMDB.class.getName()).log(Level.SEVERE, null, ex);
                    }
                }
            });
            t.setDaemon(true);
            t.start();
            Thread.sleep(10000);
            System.out.println("Lanzando MDB...");
            launchMDB();
            Thread.sleep(20000);
            System.out.println("Iniciando bucle...");
            threadLoop();
        } catch (InterruptedException ex) {
            Logger.getLogger(ROSMDB.class.getName()).log(Level.SEVERE, null, ex);
        }
        
        subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
            @Override
            public void onNewMessage(std_msgs.String message) {
                System.out.println("\n\nrosmdb MESSAGE: "+message.getData());
                String option = message.getData();
                switch (option){
                    case "stop":
                        try{
                            System.out.println("\n\nCerrando ROSMDB\n\n");
                            boolean run = getRunning();
                            run = false;
                            subscriber.shutdown();
                            cleanNode();
                            ConnectedNode cn = getNode();
                            cn.shutdown();
                            break;
                        }catch (IllegalStateException ise){
                            System.out.println(ise);
                        }catch (RosRuntimeException rre){
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
        Iterator<String> names = actuatorSockets.keySet().iterator();
        for (final SocketReader sr : actuatorSockets.values()) {
            final String name = names.next();
            Thread t = new Thread(new Runnable() {
                @Override
                public void run() {
                    while(running) {
                        System.out.println("Esperando a recibir de: " + name);
                        double readed = sr.readDouble();
                        log.info("Recibido de actuador (MDB) " + name + " " + readed + " (publicando por ros)");
                        System.out.println("Recibido de actuador (MDB) " + name + " " + readed + " (publicando por ros)");
                        std_msgs.String str = (std_msgs.String) actuatorPublishers.get(name).newMessage();
                        str.setData("" + readed);
                        actuatorPublishers.get(name).publish(str);
                    }
                }
            });
            t.setDaemon(true);
            t.start();
        }
    }
}
