use std::sync::mpsc;

#[derive(Clone, Copy)]
pub enum Command {
    Work,
    Pause,
    Abort,
}

pub const PAUSE_LOG: &str = "##**PAUSED**##";

pub struct LocalManager {
    commands: mpsc::Receiver<Command>,
    log: mpsc::Sender<String>,
}

impl LocalManager {
    fn update_impl(&mut self, log_msg: String) -> Result<(), String> {
        self.log.send(log_msg).ok();

        let mut new_command = None;
        loop {
            while let Ok(cmd) = self.commands.try_recv() {
                new_command = Some(cmd);
            }
            if let Some(cmd) = new_command {
                match cmd {
                    Command::Abort => {
                        return Err("manuell abgebrochen".into());
                    },
                    Command::Pause => {
                        //pause for short time, then redo check via endless loop.
                        self.log.send(PAUSE_LOG.into()).ok();
                        std::thread::sleep(std::time::Duration::from_secs(1));
                    },
                    Command::Work => {
                        return Ok(());
                    },
                }
            } else {
                return Ok(());
            }
        }
    }

    pub fn update(&mut self, msg: impl Into<String>) -> Result<(), String> {
        self.update_impl(msg.into())
    }
}

pub struct ExternManager {
    commands: mpsc::Sender<Command>,
    log: mpsc::Receiver<String>,
    last_log: String,
    paused: bool,
}

impl ExternManager {
    pub fn send_command(&mut self, cmd: Command) -> Result<(), mpsc::SendError<Command>> {
        if matches!(cmd, Command::Work | Command::Abort) {
            self.paused = false;
        }
        self.commands.send(cmd)
    }

    pub fn update(&mut self) {
        while let Ok(log) = self.log.try_recv() {
            if log == PAUSE_LOG {
                self.paused = true;
            } else {
                self.last_log = log;
            }
        }
    }

    pub fn last_log(&self) -> &str {
        &self.last_log
    }

    pub fn is_paused(&self) -> bool {
        self.paused
    }
}

pub fn build_managers() -> (ExternManager, LocalManager) {
    let (cmd_send, cmd_rcv) = mpsc::channel();
    let (log_send, log_rcv) = mpsc::channel();
    let local = LocalManager {
        commands: cmd_rcv,
        log: log_send,
    };
    let extrn = ExternManager {
        commands: cmd_send,
        log: log_rcv,
        last_log: String::new(),
        paused: false,
    };
    (extrn, local)
}
