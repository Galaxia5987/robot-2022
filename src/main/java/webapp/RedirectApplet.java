package webapp;

import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import java.io.IOException;

public class RedirectApplet extends HttpServlet {
	private static final long serialVersionUID = -3919085456423945671L;

	@Override
	protected void doGet(HttpServletRequest req, HttpServletResponse res) throws IOException {
		res.sendRedirect("/firescope.html");
	}
}
