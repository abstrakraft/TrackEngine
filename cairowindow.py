import pygtk
pygtk.require('2.0')
import gtk
import threading

# Create a GTK+ widget on which we will draw using Cairo
class CairoWindowSurface(gtk.DrawingArea):
	def __init__(self):
		gtk.DrawingArea.__init__(self)
		self.events = {"delete-event": gtk.main_quit}

	def run(self):
		gt = self.GTKThread(self)
		gt.run()

	# Draw in response to an expose-event
	__gsignals__ = { "expose-event": "override" }

	# Handle the expose-event by drawing
	def do_expose_event(self, event):
		# Create the cairo context
		cr = self.window.cairo_create()

		# Restrict Cairo to the exposed area; avoid extra work
		cr.rectangle(event.area.x, event.area.y,
		event.area.width, event.area.height)
		cr.clip()

		self.draw(cr, *self.window.get_size())

	def draw(self, cr, width, height):
		# Fill the background with white
		cr.set_source_rgb(1.0, 1.0, 1.0)
		cr.rectangle(0, 0, width, height)
		cr.fill()

	class GTKThread(threading.Thread):
		def __init__(self, widget):
			threading.Thread.__init__(self)
			self.widget = widget

		def run(self):
			window = gtk.Window()
			for (k,v) in self.widget.events.items():
				window.connect(k, v)
			self.widget.show()
			window.add(self.widget)
			window.present()
			gtk.main()

# GTK mumbo-jumbo to show the widget in a window and quit when it's closed
def run(widget):
	window = gtk.Window()
	window.connect("delete-event", gtk.main_quit)
	#window.connect("key-press-event", key_handler)
	widget.show()
	window.add(widget)
	window.present()
	gtk.main()
