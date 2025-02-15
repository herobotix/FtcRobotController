/*import java.util.ArrayList;

public class Main {
  static void Toggler(ArrayList<Integer> iList) {
	int tState = iList.get(0);
	boolean tButton = (iList.get(2)==1);
	iList.set(0,
		(tState==0 && !tButton)?(1):(
			(tState==1 && tButton)?(2):(
				(tState==2 && !tButton)?(3):(
					(tState==3 && tButton)?(0):(tState)
				)
			)
		)
	);
    System.out.println(tState);
    
  }

  public static void main(String[] args) {
 	ArrayList<Integer> vList = new ArrayList<Integer>();
    vList.add(0);
    vList.add(0);
    vList.add(0);
    Toggler(vList);
    System.out.println(vList);
  }
}
*/