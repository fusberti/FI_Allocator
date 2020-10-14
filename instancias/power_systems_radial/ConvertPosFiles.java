import java.awt.Point;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class ConvertPosFiles{
	private static BufferedReader reader;
	private static BufferedWriter writer;
	
	public static void openFiles(String inName, String outName) throws IOException {
		reader = new BufferedReader(new FileReader(inName));
//		writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(outName), "utf-8"));
		writer = new BufferedWriter(new FileWriter(outName));
	}
	
	public static void convertFile() throws IOException {
		reader.readLine();reader.readLine();reader.readLine();
		String line = reader.readLine();
		String[] words = line.split("\\s+");
		int totalArcs = Integer.parseInt(words[1]);
		line = reader.readLine();
		words = line.split("\\s+");
		int numArcs = Integer.parseInt(words[1]);
		reader.readLine();
		line = reader.readLine();
		words = line.split("\\s+");
		int numNodes = Integer.parseInt(words[1]);
		reader.readLine();
		line = reader.readLine();
		words = line.split("\\s+");
		String feeders = "";
		for(int i=1; i<words.length; i++)
			feeders += "\t" + words[i] ;
		reader.readLine();
		ArrayList<String> arcsInfo = new ArrayList<>();
		for(int i=0; i<numArcs; i++) {
			line = reader.readLine();
			words = line.split("\\s+");
			arcsInfo.add(words[1] + "\t" + words[2] );
		}
		reader.readLine();
		for(int i=0; i<totalArcs-numArcs; i++) {
			reader.readLine();
		}
		reader.readLine();reader.readLine();
		writer.write("p\tfi\t" + numNodes + "\t" + numArcs + "\t" + 5 + "\t" + 0.5 + "\n");
		writer.write("f" + feeders + "\n");
		Map<Integer, Point> mapNodes = new HashMap<Integer, Point>();
		for(int i=0; i<numNodes; i++) {
			line = reader.readLine();
			words = line.split("\\s+");
			writer.write("v\t" + words[0] + "\t" + words[1] + "\t" +  words[2] + "\n");
			mapNodes.put(Integer.parseInt(words[0]), new Point(Integer.parseInt(words[1]), Integer.parseInt(words[2])));
		}		
		for(String info : arcsInfo) {
			String[] nodes = info.split("\t");
			int source = Integer.parseInt(nodes[0]);
			int target = Integer.parseInt(nodes[1]);
			double dist = 0.1*Math.sqrt(Math.pow(mapNodes.get(source).getX()-mapNodes.get(target).getX(),2) + Math.pow(mapNodes.get(source).getY()-mapNodes.get(target).getY(),2));
			writer.write("e\t" + info + "\t" + dist + "\n");
		}
		reader.close();
		writer.close();
	}
	
	public static void main(String[] args) throws IOException {
		// TODO Auto-generated method stub
		String outFile = args[0].substring(0, args[0].lastIndexOf('.')) + ".fi";
		openFiles(args[0], outFile);
		convertFile();
		System.out.println("Conversion done!");
	}

}
