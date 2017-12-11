use std::io::{Read, Write};
use std::boxed::FnBox;

use serde;
use serde_json::{Error, Serializer, Deserializer};
use serde_json::de::IoRead as JsonRead;

use gfx;
use common::{Common, CommonReply};

pub mod halo;
// pub mod home;
pub mod lets_get_physical;
pub mod snowflakes;
// pub mod workshop;
pub mod settings;

pub trait App<R: gfx::Resources, C: gfx::CommandBuffer<R>, W: Write, Re: Read> {
    fn update<'a>(&'a mut self, common: &mut Common<R, C>) -> Box<FnBox(&mut CommonReply<R, C>) + 'a>;

    fn se_state(&self, serializer: &mut Serializer<W>) -> Result<<&mut Serializer<W> as serde::Serializer>::Ok, Error>;
    fn de_state(&mut self, deserializer: Deserializer<JsonRead<Re>>) -> Result<(), Error>;
}
