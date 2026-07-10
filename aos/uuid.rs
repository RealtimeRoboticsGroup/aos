#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct UUID {
    pub data: [u8; 16],
}

impl From<UUID> for uuid::Uuid {
    fn from(uuid: UUID) -> uuid::Uuid {
        uuid::Uuid::from_bytes(uuid.data)
    }
}

impl AsRef<uuid::Uuid> for UUID {
    fn as_ref(&self) -> &uuid::Uuid {
        uuid::Uuid::from_bytes_ref(&self.data)
    }
}

impl From<uuid::Uuid> for UUID {
    fn from(uuid: uuid::Uuid) -> UUID {
        UUID {
            data: *uuid.as_bytes(),
        }
    }
}
