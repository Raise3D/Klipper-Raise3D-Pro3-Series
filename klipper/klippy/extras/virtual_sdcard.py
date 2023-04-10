# Virtual sdcard support (print files directly from a host g-code file)
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
# from genericpath import exists
# from importlib.resources import path
# -*- coding: utf-8 -*-
from datetime import datetime
import math
import os, logging
import json
import time
from setuptools import Command

VALID_GCODE_EXTS = ['gcode', 'GCODE', 'g', 'gco']
POWERLOSS_FILE_NAME = "/opt/Raise3D/gcode-cache/klipper-print.powerloss"

class VirtualSD:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        printer.register_event_handler("klippy:shutdown", self.handle_shutdown)
        printer.register_event_handler("hotend:missing", self.handle_hotend_missing)
        printer.register_event_handler("temperature:error", self.handle_shutdown)
        # sdcard state
        sd = config.get('path')
        self.sdcard_dirname = os.path.normpath(os.path.expanduser(sd))
        self.current_file = None
        self.file_position = self.file_size = 0
        self.print_progress = 0
        self.printed_lineno = 0
        # Print Stat Tracking
        self.print_stats = printer.load_object(config, 'print_stats')
        # Work timer
        self.reactor = printer.get_reactor()
        self.must_pause_work = self.cmd_from_sd = False
        self.next_file_position = 0
        self.work_timer = None
        # Register commands
        self.gcode = printer.lookup_object('gcode')
        for cmd in ['M20', 'M21', 'M23', 'M24', 'M25', 'M26', 'M27', 'M32', 'M38', 'M39']:
            self.gcode.register_command(cmd, getattr(self, 'cmd_' + cmd))
        for cmd in ['M28', 'M29', 'M30']:
            self.gcode.register_command(cmd, self.cmd_error)
        self.gcode.register_command(
            "SDCARD_RESET_FILE", self.cmd_SDCARD_RESET_FILE,
            desc=self.cmd_SDCARD_RESET_FILE_help)
        self.gcode.register_command(
            "SDCARD_PRINT_FILE", self.cmd_SDCARD_PRINT_FILE,
            desc=self.cmd_SDCARD_PRINT_FILE_help)
        # printer.register_event_handler("filament:in", self.filament_in_event)
        # printer.register_event_handler("filament:out", self.filament_out_event)
        printer.register_event_handler("ghead:hotend", self.hotend_handle)
        self.printed_pos_z = 0
        self.powerloss_record_time = datetime.now()
        self.is_recover_print = False
        self.record = {'MODIFYTIME':time.time(), 'SEEKPOSITION':0}
        self.is_start_move = False
        self.powerloss_file_list = [POWERLOSS_FILE_NAME+".0", POWERLOSS_FILE_NAME+".1", POWERLOSS_FILE_NAME+".2", POWERLOSS_FILE_NAME+".3", POWERLOSS_FILE_NAME+".4"]
        self.powerloss_file_index = 0
    
    def hotend_handle(self, hotend_ptr, val):
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead != None and self.work_timer != None:
            active_extruder = toolhead.get_extruder()
            if (active_extruder == 'extruder' and hotend_ptr == 'left') and (
                active_extruder == 'extruder1' and hotend_ptr == 'right'):
                if val == 1:
                    self.do_pause("%s hotend is off" % hotend_ptr)
                pass
        pass
    
    def filament_out_event(self, filament_n):
        # check if the filament ch match active extruder !!;
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead != None and self.work_timer != None:
            active_extruder = toolhead.get_extruder()
            if (active_extruder == 'extruder' and filament_n == 'F0') and (
                active_extruder == 'extruder1' and filament_n == 'F1'):
                self.do_pause("Filament run out")
                pass
        pass
    
    def filament_in_event(self, filament_n):
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead != None and self.work_timer != None:
            active_extruder = toolhead.get_extruder()
            if (active_extruder == 'extruder' and filament_n == 'F0') and (
                active_extruder == 'extruder1' and filament_n == 'F1'):
                if self.current_file is not None and self.must_pause_work:
                    self.do_resume("Filament in")
                pass
        pass
    
    def handle_shutdown(self):
        if self.work_timer is not None:
            self.must_pause_work = True
            try:
                readpos = max(self.file_position - 1024, 0)
                readcount = self.file_position - readpos
                self.current_file.seek(readpos)
                data = self.current_file.read(readcount + 128)
            except:
                logging.exception("virtual_sdcard shutdown read")
                return
            logging.info("Virtual sdcard (%d): %s\nUpcoming (%d): %s",
                         readpos, repr(data[:readcount]),
                         self.file_position, repr(data[readcount:]))

    def handle_hotend_missing(self, missing_hotend):
        toolhead = self.printer.lookup_object('toolhead')
        self.gcode.respond_raw("handle hotend missing %s" % (missing_hotend))
        if self.work_timer != None and toolhead != None:
            active_extruder = toolhead.get_extruder()
            self.gcode.respond_raw("handle hotend missing222 %s %s" % (active_extruder.get_name(), missing_hotend))
            if (active_extruder.get_name() == 'extruder' and missing_hotend == 'left') or (active_extruder.get_name() == 'extruder1' and missing_hotend == 'right'):
                self.must_pause_work = True
                self.gcode._respond_error("%s hotend missing, stop print", missing_hotend)
                # logging.warning("%s hotend missing, stop print", missing_hotend)
    
    def stats(self, eventtime):
        if self.work_timer is None:
            return False, ""
        return True, "sd_pos=%d" % (self.file_position,)
    
    def get_file_list(self, check_subdirs=False):
        if check_subdirs:
            flist = []
            for root, dirs, files in os.walk(
                    self.sdcard_dirname, followlinks=True):
                for name in files:
                    ext = name[name.rfind('.') + 1:]
                    if ext not in VALID_GCODE_EXTS:
                        continue
                    full_path = os.path.join(root, name)
                    r_path = full_path[len(self.sdcard_dirname) + 1:]
                    size = os.path.getsize(full_path)
                    flist.append((r_path, size))
            return sorted(flist, key=lambda f: f[0].lower())
        else:
            dname = self.sdcard_dirname
            try:
                filenames = os.listdir(self.sdcard_dirname)
                return [(fname, os.path.getsize(os.path.join(dname, fname)))
                        for fname in sorted(filenames, key=str.lower)
                        if not fname.startswith('.')
                        and os.path.isfile((os.path.join(dname, fname)))]
            except:
                logging.exception("virtual_sdcard get_file_list")
                raise self.gcode.error("Unable to get file list")
    
    def get_status(self, eventtime):
        return {
            'file_path': self.file_path(),
            'progress': self.progress(),
            'is_active': self.is_active(),
            'file_position': self.file_position,
            'file_size': self.file_size,
        }
    
    def file_path(self):
        if self.current_file:
            return self.current_file.name
        return None
    
    def progress(self):
        if self.file_size:
            return float(self.file_position) / self.file_size
        else:
            return 0.
    
    def is_active(self):
        return self.work_timer is not None
    
    def do_pause(self, pause_reason=None):
        if self.work_timer is not None:
            self.must_pause_work = True
            self.printer.send_event("virtul_sd:status",False)
            if pause_reason != None:
                self.gcode._respond_warning(str(pause_reason))
            while self.work_timer is not None and not self.cmd_from_sd:
                self.reactor.pause(self.reactor.monotonic() + .001)
                self.powerloss_record_time = datetime.now()
    
    def do_resume(self, reason=None):
        if self.work_timer is not None:
            raise self.gcode.error("SD busy")
        self.must_pause_work = False
        if reason != None:
            self.gcode.respond_raw(str(reason))
        self.printer.send_event("virtul_sd:status",True)
        self.work_timer = self.reactor.register_timer(
            self.work_handler, self.reactor.NOW)
        self.powerloss_record_time = datetime.now()
    
    def do_recover(self, reason=None):
        if self.work_timer is not None:
            raise self.gcode.error("SD busy")
        self.must_pause_work = False
        if reason != None:
            self.gcode.respond_raw(str(reason))
        self.work_timer = self.reactor.register_timer(
            self.work_handler, self.reactor.NOW)
        self.powerloss_record_time = datetime.now()
    
    def do_cancel(self):
        if self.current_file is not None:
            self.do_pause()
            self.current_file.close()
            self.current_file = None
            self.print_stats.note_cancel()
        self.file_position = self.file_size = 0.
        self.powerloss_record_time = datetime.now()
        self.printer.send_event("virtul_sd:status",False)
    
    # G-Code commands
    def cmd_error(self, gcmd):
        raise gcmd.error("SD write not supported")
    
    def _reset_file(self):
        if self.current_file is not None:
            self.do_pause()
            self.current_file.close()
            self.current_file = None
        self.file_position = self.file_size = 0.
        self.print_stats.reset()
    
    cmd_SDCARD_RESET_FILE_help = "Clears a loaded SD File. Stops the print " \
        "if necessary"
    
    def cmd_SDCARD_RESET_FILE(self, gcmd):
        if self.cmd_from_sd:
            raise gcmd.error(
                "SDCARD_RESET_FILE cannot be run from the sdcard")
        self._reset_file()
        gcmd.respond_raw("Print stopped")
        # STOP all heater!!
    
    cmd_SDCARD_PRINT_FILE_help = "Loads a SD file and starts the print.  May " \
        "include files in subdirectories."
    
    def cmd_SDCARD_PRINT_FILE(self, gcmd):
        if self.work_timer is not None:
            raise gcmd.error("SD busy")
        self._reset_file()
        filename = gcmd.get("FILENAME")
        if filename[0] == '/':
            filename = filename[1:]
        self._load_file(gcmd, filename, check_subdirs=True)
        self.do_resume()
    
    def cmd_M20(self, gcmd):
        # List SD card
        files = self.get_file_list()
        gcmd.respond_raw("Begin file list")
        for fname, fsize in files:
            gcmd.respond_raw("%s %d" % (fname, fsize))
        gcmd.respond_raw("End file list")
    
    def cmd_M38(self, gcmd):
        if self.work_timer is not None:
            raise gcmd.error("SD busy")
        gcmd.respond_raw(str(gcmd._params))
        filename = gcmd.get('P').decode("utf-8")
        self.is_start_move = False

        if '/' in filename:
            if not os.path.exists(filename):
                gcmd.respond_raw("warning : File %s not exsit!" % filename)
                return
            files = filename.rsplit("/", 1)
            ext = filename[filename.rfind('.') + 1:]
            if ext not in VALID_GCODE_EXTS:
            # if len(files) != 2 or not files[1].endswith('gcode'):
                gcmd.respond_raw("warning : Ivalide file type!" % filename)
                return 
            self.sdcard_dirname = os.path.normpath(files[0])
            self._load_file(gcmd, files[1], check_subdirs=True)
            self.do_resume()
        else:
            # complete_file = "%s/%s"%(self.sdcard_dirname,filename)
            # if not os.path.exists(complete_file):
            cfile = filename
            gcmd.respond_raw("filename = %s" % cfile)
            #     return
            self._load_file(gcmd, cfile, check_subdirs=True)
            self.do_resume()

    def cmd_M39(self, gcmd):
        if self.work_timer is not None:
            raise gcmd.error("SD busy")
        try:
            orig = gcmd.get_commandline()
            command_line = orig[orig.find("M39") + 4:].split()[0].strip()
            if '*' in command_line:
                cmd_msg = command_line.split("*")
                filename = str(cmd_msg[0]).strip()
                filepos  = int(cmd_msg[1])
                gcmd.respond_raw("file_name = %s filepos = %d"%(filename,filepos))
            else:
                gcmd.respond_error("cmd should has pos parameter")
                return
        except:
            raise gcmd.error("Unable to extract filename")
            # check if the file is exist
        if not os.path.exists(filename):
            gcmd.respond_raw("Error: File %s not exsit!" % filename)
            return
        files = filename.rsplit("/", 1)
        if len(files) != 2 or not files[1].endswith('gcode'):
            gcmd.respond_raw("warning : Ivalide file type!" % filename)
            return
        self.print_progress = 0
        self.sdcard_dirname = os.path.normpath(files[0])
        self._load_file(gcmd, files[1], check_subdirs=True)
        if '*' in command_line:
            #self.file_position = int(command_line[command_line.find('*') + 1:].strip())
            self.file_position = filepos
            # gcmd.respond_raw("get recover print %s %d" %(filename, self.file_position,))
            # return
        try:
            self.current_file.seek(self.file_position)
        except:
            logging.exception("virtual_sdcard recover seek")
        self.do_resume()
        self.is_recover_print = True
        gcmd.respond_raw("get recover print %s %d" % (filename, self.file_position))

    def cmd_M32(self, gcmd):
        if self.work_timer is not None:
            raise gcmd.error("SD busy")
        try:
            orig = gcmd.get_commandline()
            if "M32 filename=" in orig:
                filename = orig[orig.find("M32 filename=")].split()[0].strip()
            else:
                filename = orig[orig.find("M32") + 4:].split()[0].strip()
            #filename = orig[orig.find("M32 filename=")].split()[0].strip()
            if '*' in filename:
                filename = filename[:filename.find('*')].strip()
        except:
            raise gcmd.error("Unable to extract filename")
        # check if the file is exist
        if not os.path.exists(filename):
            gcmd.respond_raw("Error: File %s not exsit!" % filename)
            return
        files = filename.rsplit("/", 1)
        ext = filename[filename.rfind('.') + 1:]
        if ext not in VALID_GCODE_EXTS:
        # if len(files) != 2 or not files[1].endswith('gcode'):
            gcmd.respond_raw("warning : Ivalide file type!" % filename)
            return 
        self.print_progress = 0
        self.printed_lineno = 0
        self.sdcard_dirname = os.path.normpath(files[0])
        self._load_file(gcmd, files[1], check_subdirs=True)
        self.start_record(filename,self.file_size)
        gcmd.respond_raw("start print")
        self.is_start_move = False
        self.do_resume()
    
    def start_record(self,file_name,file_size):
        self.powerloss_file_index = 0
        self.record["MODIFYTIME"] = time.time()
        for powerloss_file in self.powerloss_file_list:
            with open(powerloss_file, "w") as json_file:
                json.dump(self.record, json_file)
    
    def record_file_position(self,position,line):
        self.record["MODIFYTIME"] = time.time()
        self.record["SEEKPOSITION"] = position
        self.record["RECORD_LINE"] = line
        powerloss_file = POWERLOSS_FILE_NAME + ".%d" %(self.powerloss_file_index)
        with open(powerloss_file, "w") as json_file:
            json.dump(self.record, json_file)

        self.powerloss_file_index += 1
        if self.powerloss_file_index >= len(self.powerloss_file_list) :
            self.powerloss_file_index = 0
    
    def complete_record(self):
        if os.path.exists(POWERLOSS_FILE_NAME):
            deleteCommand = "rm -rf " + POWERLOSS_FILE_NAME
            os.system(deleteCommand)
        pass
    
    def cmd_M21(self, gcmd):
        # Initialize SD card
        gcmd.respond_raw("SD card ok")
    
    def cmd_M23(self, gcmd):
        # Select SD file
        if self.work_timer is not None:
            raise gcmd.error("SD busy")
        self._reset_file()
        try:
            orig = gcmd.get_commandline()
            filename = orig[orig.find("M23") + 4:].split()[0].strip()
            if '*' in filename:
                filename = filename[:filename.find('*')].strip()
        except:
            raise gcmd.error("Unable to extract filename")
        if filename.startswith('/'):
            filename = filename[1:]
        self._load_file(gcmd, filename)
    
    def _get_file_notes_size(self, filename):
        f = open(filename, 'rb')
        line = f.readline()
        note_size = 0
        while line:
            if line.startswith(';'):
                note_size += len(line)
            line = f.readline()
        f.close()
        return note_size
    
    def _load_file(self, gcmd, filename, check_subdirs=False):
        files = self.get_file_list(check_subdirs)
        flist = [f[0] for f in files]
        files_by_lower = {fname.lower(): fname for fname, fsize in files}
        fname = filename
        try:
            if fname not in flist:
                fname = files_by_lower[fname.lower()]
            fname = os.path.join(self.sdcard_dirname, fname)
            # note_size = self._get_file_notes_size(self, fname)
            f = open(fname, 'rb')
            f.seek(0, os.SEEK_END)
            fsize = f.tell()
            f.seek(0)
        except:
            logging.exception("virtual_sdcard file open")
            raise gcmd.error("Unable to open file")
        gcmd.respond_raw("File opened:%s Size:%d" % (filename, fsize))
        gcmd.respond_raw("File selected")
        gcmd.respond_raw("Print started")
        self.current_file = f
        self.file_position = 0
        self.file_size = fsize
        self.print_stats.set_current_file(filename)
    
    def cmd_M24(self, gcmd):
        # Start/resume SD print
        self.do_resume()
        gcmd.respond_raw("Print resumed")
   
    def cmd_M25(self, gcmd):
        # Pause SD print
        self.do_pause()
        gcode_move = self.printer.lookup_object('gcode_move')
        printspeed = gcode_move._get_gcode_speed()
        gcmd.respond_raw("Print paused - speed: %.2f" %(printspeed))
    
    def cmd_M26(self, gcmd):
        # Set SD position
        if self.work_timer is not None:
            raise gcmd.error("SD busy")
        pos = gcmd.get_int('S', minval=0)
        self.file_position = pos
    
    def cmd_M27(self, gcmd):
        # Report SD print status
        if self.current_file is None:
            gcmd.respond_raw("Not SD printing.")
            return
        gcmd.respond_raw("SD printing byte %d/%d"
                         % (self.file_position, self.file_size))
    
    def get_file_position(self):
        return self.next_file_position
    
    def set_file_position(self, pos):
        self.next_file_position = pos
    
    def is_cmd_from_sd(self):
        return self.cmd_from_sd
    
    # Background work timer
    def work_handler(self, eventtime):
        logging.info("Starting SD card print (position %d)", self.file_position)
        self.reactor.unregister_timer(self.work_timer)
        try:
            self.current_file.seek(self.file_position)
        except:
            logging.exception("virtual_sdcard seek")
            self.work_timer = None
            return self.reactor.NEVER
        self.print_stats.note_start()
        gcode_mutex = self.gcode.get_mutex()
        partial_input = ""
        lines = []
        error_message = None
        data_flag = False
        while not self.must_pause_work:
            if not lines:
                # Read more data
                try:
                    data = self.current_file.read(8192)
                except:
                    logging.exception("virtual_sdcard read")
                    break
                if not data:
                    # End of file
                    self.current_file.close()
                    self.current_file = None
                    logging.info("Finished SD card print")
                    self.gcode.respond_raw("Done printing file")
                    self.printer.send_event("virtul_sd:status",False)
                    self.complete_record()
                    break
                lines = data.split('\n')
                lines[0] = partial_input + lines[0]
                partial_input = lines.pop()
                lines.reverse()
                self.reactor.pause(self.reactor.NOW)
                continue
            # Pause if any other request is pending in the gcode class
            if gcode_mutex.test():
                self.reactor.pause(self.reactor.monotonic() + 0.100)
                continue
            # Dispatch command
            self.cmd_from_sd = True
            line = lines.pop()
            if line.startswith(";Data start"):
                data_flag = True
            if line.startswith(";Data end"):
                data_flag = False
            next_file_position = self.file_position + len(line) + 1
            self.next_file_position = next_file_position
            if self.is_recover_print:
                self.is_recover_print = False
                self.gcode.respond_raw("recover command line %s"%line)
            try:
                if (not line.startswith(';') or line.startswith(";LAYER:") or line.startswith("LAYER:")) and not line.startswith("M99123"):
                    self.printed_lineno += 1
                    self.gcode.run_script(line) 
                    # self.gcode.respond_raw("print line:%s"%(line))
                if not self.is_start_move and (line.startswith('G1') or line.startswith('G0')) :
                    self.gcode.respond_raw("Print move start: %s"%(line))
                    self.is_start_move = True
                if line.startswith("M2000"):
                    self.gcode.respond_raw("Printing pause by Gcode")

            except self.gcode.error as e:
                logging.exception("virtual_sdcard dispatch")
                error_message = str(e)
                break
            self.cmd_from_sd = False
            self.file_position = self.next_file_position
            
            # 100 ms record powerloss file
            record_timeout = (datetime.now() - self.powerloss_record_time).microseconds / 1000
            if self.is_start_move and (record_timeout > 100 or line.startswith(';Z:')):
                self.record_file_position(self.file_position, line)
                self.powerloss_record_time = datetime.now()

            # cal print progress
            if not data_flag:
                temp_print_progress = self.file_position * 1.0 / self.file_size * 100.0
                if (math.fabs(self.print_progress - temp_print_progress) > 0.1) or (temp_print_progress >= 100.0):
                    self.print_progress = temp_print_progress
                    self.gcode.respond_raw("SD printing progress %d/%d %d" % (self.file_position, self.file_size, self.printed_lineno))
            # Do we need to skip around?
            if self.next_file_position != next_file_position:
                try:
                    self.current_file.seek(self.file_position)
                except:
                    logging.exception("virtual_sdcard seek")
                    self.work_timer = None
                    return self.reactor.NEVER
                lines = []
                partial_input = ""
        
        logging.info("Exiting SD card print (position %d)", self.file_position)
        self.work_timer = None
        self.cmd_from_sd = False
        if error_message is not None:
            self.print_stats.note_error(error_message)
        elif self.current_file is not None:
            self.print_stats.note_pause()
        else:
            self.print_stats.note_complete()
        return self.reactor.NEVER


def load_config(config):
    return VirtualSD(config)
